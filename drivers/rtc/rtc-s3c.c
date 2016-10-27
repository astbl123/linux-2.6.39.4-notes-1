/* drivers/rtc/rtc-s3c.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Copyright (c) 2004,2006 Simtec Electronics
 *	Ben Dooks, <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410/S3C2440/S3C24XX Internal RTC Driver
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <plat/regs-rtc.h>

enum s3c_cpu_type {
	TYPE_S3C2410,
	TYPE_S3C64XX,
};

/* I have yet to find an S3C implementation with more than one
 * of these rtc blocks in */

static struct resource *s3c_rtc_mem;

static struct clk *rtc_clk;
static void __iomem *s3c_rtc_base;
static int s3c_rtc_alarmno = NO_IRQ;
static int s3c_rtc_tickno  = NO_IRQ;
static bool wake_en;
static enum s3c_cpu_type s3c_rtc_cpu_type;

static DEFINE_SPINLOCK(s3c_rtc_pie_lock);

/* IRQ Handlers */

static irqreturn_t s3c_rtc_alarmirq(int irq, void *id)
{
	struct rtc_device *rdev = id;

	rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);

	if (s3c_rtc_cpu_type == TYPE_S3C64XX)
		writeb(S3C2410_INTP_ALM, s3c_rtc_base + S3C2410_INTP);

	return IRQ_HANDLED;
}

static irqreturn_t s3c_rtc_tickirq(int irq, void *id)
{
	struct rtc_device *rdev = id;

	rtc_update_irq(rdev, 1, RTC_PF | RTC_IRQF);

	if (s3c_rtc_cpu_type == TYPE_S3C64XX)
		writeb(S3C2410_INTP_TIC, s3c_rtc_base + S3C2410_INTP);

	return IRQ_HANDLED;
}

/**
 * 设置是否允许脉冲中断[1]P221.
 *
 * @param dev -- RTC设备结构体[1]P221.
 * @param enabled 
 * - 1 -- 允许
 * - 0 -- 不允许
 *
 * Update control registers 
 * 
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_setaie(struct device *dev, unsigned int enabled)
{
	unsigned int tmp; 		/* 声明一个局部变量[1]P221. */

	pr_debug("%s: aie=%d\n", __func__, enabled);

	tmp = readb(s3c_rtc_base + S3C2410_RTCALM) & ~S3C2410_RTCALM_ALMEN;

	if (enabled)
		tmp |= S3C2410_RTCALM_ALMEN;

	writeb(tmp, s3c_rtc_base + S3C2410_RTCALM);

	return 0;
}

/**
 * 设置时钟脉冲中断频率[1]P211.
 * 
 * @param dev -- RTC设备[1]P212. 
 * @param freq -- 频率,若为1, 表示1秒产生一次中断; 若为2, 表示1秒产生2次
 * 				  中断...[1]P212
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_setfreq(struct device *dev, int freq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);
	unsigned int tmp = 0;

	if (!is_power_of_2(freq)) 				/* 判断需要设置的频率是不是2的倍数[1]P212. */
		return -EINVAL;

	spin_lock_irq(&s3c_rtc_pie_lock); 		/* 锁定自旋锁[1]P212. */

	if (s3c_rtc_cpu_type == TYPE_S3C2410) {
		tmp = readb(s3c_rtc_base + S3C2410_TICNT); 	/* 读取TICNT寄存器中的值, S3C2410_TICNT是这个寄存器的地址[1]P212. */
		tmp &= S3C2410_TICNT_ENABLE; 				/* 将最后7位设置为零[1]P212. */
	}

	tmp |= (rtc_dev->max_user_freq / freq)-1; 		/* 设置TICNT低7位的值[1]P212. */

	writel(tmp, s3c_rtc_base + S3C2410_TICNT); /* 将设置的值写入TICNT寄存器[1]P212. */
	spin_unlock_irq(&s3c_rtc_pie_lock); 	/* 解除自旋锁[1]P212. */

	return 0;
}

/**
 * 当调用read()函数时, 会间接的调用该函数, 以获得实时时钟的时间[1]P216.
 *
 * 时间值分别保存在RTC实时时钟的各个寄存器中: 秒寄存器(BCDSEC), 日期寄存
 * 器(SECDATA), 分钟寄存器(BCDMIN)和小时寄存器(BCDHOUR)[1]P216
 *
 * Time read/write 
 * 
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned int have_retried = 0;
	void __iomem *base = s3c_rtc_base;

 retry_get_time:
	rtc_tm->tm_min  = readb(base + S3C2410_RTCMIN);
	rtc_tm->tm_hour = readb(base + S3C2410_RTCHOUR);
	rtc_tm->tm_mday = readb(base + S3C2410_RTCDATE);
	rtc_tm->tm_mon  = readb(base + S3C2410_RTCMON);
	rtc_tm->tm_year = readb(base + S3C2410_RTCYEAR);
	rtc_tm->tm_sec  = readb(base + S3C2410_RTCSEC);

	/* the only way to work out wether the system was mid-update
	 * when we read it is to check the second counter, and if it
	 * is zero, then we re-try the entire read
	 */

	if (rtc_tm->tm_sec == 0 && !have_retried) {  	/* 如果秒寄存器的值是0, 则表示过了1分钟, */
		have_retried = 1; 							/* 那么小时, 天和月等寄存器的值都可能已经  */
		goto retry_get_time; 						/* 发生了变化, 则重读一次[1]P218. */
	}

	pr_debug("read time %04d.%02d.%02d %02d:%02d:%02d\n",
		 1900 + rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
		 rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);

	rtc_tm->tm_sec = bcd2bin(rtc_tm->tm_sec);
	rtc_tm->tm_min = bcd2bin(rtc_tm->tm_min);
	rtc_tm->tm_hour = bcd2bin(rtc_tm->tm_hour);
	rtc_tm->tm_mday = bcd2bin(rtc_tm->tm_mday);
	rtc_tm->tm_mon = bcd2bin(rtc_tm->tm_mon);
	rtc_tm->tm_year = bcd2bin(rtc_tm->tm_year);

	rtc_tm->tm_year += 100; 	/* 将年数加100, 因为存储器中存放的是从1900年开始的时间[1]P218. */
	rtc_tm->tm_mon -= 1;

	return rtc_valid_tm(rtc_tm);
}

/**
 * 当调write()函数向设备驱动程序写入时间时, 会间接地调用
 * 该函数来设置实时时钟的时间[1]P218.
 *
 * 时间值分别保存在各个寄存器中: 秒寄存器(BCDSEC), 日期寄存
 * (SECDATA). 分钟寄存器(BCDMIN)和小时寄存器(BCDHOUR).
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	void __iomem *base = s3c_rtc_base; 	/* 将寄存器映射到虚拟内存的基地址赋给base变量[1]P218. */
	int year = tm->tm_year - 100; 		/* 存储器中存储的时间比实际时间少100年, 所以要减去100年[1]P218. */

	pr_debug("set time %04d.%02d.%02d %02d:%02d:%02d\n", 	/* 以16进制的方式打印出这些值[1]P218. */
		 1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* we get around y2k by simply not supporting it */

	if (year < 0 || year >= 100) { 		/* 由于寄存器的限制, RTC实时时钟只支持100年时间[1]P218. */
		dev_err(dev, "rtc only supports 100 years\n");
		return -EINVAL;
	}

	/* 将相应的BCD码写到寄存器中[1]P219. */
	writeb(bin2bcd(tm->tm_sec),  base + S3C2410_RTCSEC);
	writeb(bin2bcd(tm->tm_min),  base + S3C2410_RTCMIN);
	writeb(bin2bcd(tm->tm_hour), base + S3C2410_RTCHOUR);
	writeb(bin2bcd(tm->tm_mday), base + S3C2410_RTCDATE);
	writeb(bin2bcd(tm->tm_mon + 1), base + S3C2410_RTCMON);
	writeb(bin2bcd(year), base + S3C2410_RTCYEAR);

	return 0;
}

/**
 * 获取报警时间[1]P219.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *alm_tm = &alrm->time; 		/* 得到rtc_time表示的报警时间[1]P220. */
	void __iomem *base = s3c_rtc_base; 			/* 得到寄存器的虚拟地址的基地址[1]P220. */
	unsigned int alm_en; 						/* 是否使能报警[1]P220. */

	/* 从各个报警寄存器中读出其值[1]P220. */
	alm_tm->tm_sec  = readb(base + S3C2410_ALMSEC);
	alm_tm->tm_min  = readb(base + S3C2410_ALMMIN);
	alm_tm->tm_hour = readb(base + S3C2410_ALMHOUR);
	alm_tm->tm_mon  = readb(base + S3C2410_ALMMON);
	alm_tm->tm_mday = readb(base + S3C2410_ALMDATE);
	alm_tm->tm_year = readb(base + S3C2410_ALMYEAR);

	alm_en = readb(base + S3C2410_RTCALM);

	alrm->enabled = (alm_en & S3C2410_RTCALM_ALMEN) ? 1 : 0;

	pr_debug("read alarm %d, %04d.%02d.%02d %02d:%02d:%02d\n",
		 alm_en,
		 1900 + alm_tm->tm_year, alm_tm->tm_mon, alm_tm->tm_mday,
		 alm_tm->tm_hour, alm_tm->tm_min, alm_tm->tm_sec);


	/* decode the alarm enable field */

	if (alm_en & S3C2410_RTCALM_SECEN)
		alm_tm->tm_sec = bcd2bin(alm_tm->tm_sec);
	else
		alm_tm->tm_sec = -1;

	if (alm_en & S3C2410_RTCALM_MINEN)
		alm_tm->tm_min = bcd2bin(alm_tm->tm_min);
	else
		alm_tm->tm_min = -1;

	if (alm_en & S3C2410_RTCALM_HOUREN)
		alm_tm->tm_hour = bcd2bin(alm_tm->tm_hour);
	else
		alm_tm->tm_hour = -1;

	if (alm_en & S3C2410_RTCALM_DAYEN)
		alm_tm->tm_mday = bcd2bin(alm_tm->tm_mday);
	else
		alm_tm->tm_mday = -1;

	if (alm_en & S3C2410_RTCALM_MONEN) {
		alm_tm->tm_mon = bcd2bin(alm_tm->tm_mon);
		alm_tm->tm_mon -= 1;
	} else {
		alm_tm->tm_mon = -1;
	}

	if (alm_en & S3C2410_RTCALM_YEAREN)
		alm_tm->tm_year = bcd2bin(alm_tm->tm_year);
	else
		alm_tm->tm_year = -1;

	return 0;
}

/**
 * 设置报警时间[1]P220.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time; /* 得到rtc_time表示的报警时间[1]P221. */
	void __iomem *base = s3c_rtc_base; /* 得到寄存器的虚拟地址的基地址[1]P221. */
	unsigned int alrm_en; 			   /* 表示是否使能报警[1]P221. */

	/* 打印一些调试信息[1]P221. */
	pr_debug("s3c_rtc_setalarm: %d, %04d.%02d.%02d %02d:%02d:%02d\n",
		 alrm->enabled,
		 1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);


	alrm_en = readb(base + S3C2410_RTCALM) & S3C2410_RTCALM_ALMEN;
	writeb(0x00, base + S3C2410_RTCALM); 	/* 将00写入RTCALM寄存器, 使所有报警功能都打开[1]P221. */

	if (tm->tm_sec < 60 && tm->tm_sec >= 0) {
		alrm_en |= S3C2410_RTCALM_SECEN;
		writeb(bin2bcd(tm->tm_sec), base + S3C2410_ALMSEC);
	}

	if (tm->tm_min < 60 && tm->tm_min >= 0) {
		alrm_en |= S3C2410_RTCALM_MINEN;
		writeb(bin2bcd(tm->tm_min), base + S3C2410_ALMMIN);
	}

	if (tm->tm_hour < 24 && tm->tm_hour >= 0) {
		alrm_en |= S3C2410_RTCALM_HOUREN;
		writeb(bin2bcd(tm->tm_hour), base + S3C2410_ALMHOUR);
	}

	pr_debug("setting S3C2410_RTCALM to %08x\n", alrm_en); /* 打印报警功能的使能状态[1]P221. */

	writeb(alrm_en, base + S3C2410_RTCALM);

	s3c_rtc_setaie(dev, alrm->enabled);

	return 0;
}

/**
 * 可以读取proc文件系统来判断RTC实时时钟是否支持脉冲中断.
 * proc文件系统中的读取命令一般为cat命令, 会调用该函数[1]P221.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned int ticnt;

	if (s3c_rtc_cpu_type == TYPE_S3C64XX) {
		ticnt = readw(s3c_rtc_base + S3C2410_RTCCON);
		ticnt &= S3C64XX_RTCCON_TICEN;
	} else {
		ticnt = readb(s3c_rtc_base + S3C2410_TICNT);
		ticnt &= S3C2410_TICNT_ENABLE;
	}

	seq_printf(seq, "periodic_IRQ\t: %s\n", ticnt  ? "yes" : "no");
	return 0;
}

/**
 * 用户空间调用open()时, 最终会调用该函数.
 * 该函数主要申请了两个中断, 一个报警中断,
 * 一个计时中断[1]P215.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c_rtc_open(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);
	int ret;

	ret = request_irq(s3c_rtc_alarmno, s3c_rtc_alarmirq, 		/* 申请一个报警中断[1]P215. */
			  IRQF_DISABLED,  "s3c2410-rtc alarm", rtc_dev);

	if (ret) {
		dev_err(dev, "IRQ%d error %d\n", s3c_rtc_alarmno, ret);
		return ret;
	}

	ret = request_irq(s3c_rtc_tickno, s3c_rtc_tickirq,  		/* 申请一个计时中断[1]P215. */
			  IRQF_DISABLED,  "s3c2410-rtc tick", rtc_dev);

	if (ret) {
		dev_err(dev, "IRQ%d error %d\n", s3c_rtc_tickno, ret);
		goto tick_err;
	}

	return ret;

 tick_err:
	free_irq(s3c_rtc_alarmno, rtc_dev);
	return ret;
}

/*!
 * 用户空间调用close()时, 最终会调用该函数,
 * 该函数主要释放s3c_rtc_open()函数中申请的
 * 两个中断[1]P216.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c_rtc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);

	/* do not clear AIE here, it may be needed for wake */

	free_irq(s3c_rtc_alarmno, rtc_dev); 	/* 释放报警中断[1]P216. */
	free_irq(s3c_rtc_tickno, rtc_dev); 		/* 释放定时中断[1]P216. */
}

/**
 * [1]P215有提及.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static const struct rtc_class_ops s3c_rtcops = {
	.open		= s3c_rtc_open,
	.release	= s3c_rtc_release,
	.read_time	= s3c_rtc_gettime,
	.set_time	= s3c_rtc_settime,
	.read_alarm	= s3c_rtc_getalarm,
	.set_alarm	= s3c_rtc_setalarm,
	.proc		= s3c_rtc_proc,
	.alarm_irq_enable = s3c_rtc_setaie,
};

/**
 * RTC实时时钟可以设置相应的寄存器来控制实时时钟的状态, 这些状态
 * 包括使实时时钟开始工作,也包括使实时时钟停止工作[1]P210.
 *
 * @param pdev -- 平台设备
 * @en 
 * - 0 -- 关闭
 * - 1 -- 开启
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c_rtc_enable(struct platform_device *pdev, int en)
{
	void __iomem *base = s3c_rtc_base;
	unsigned int tmp;

	if (s3c_rtc_base == NULL)
		return;

	if (!en) {
		tmp = readw(base + S3C2410_RTCCON);
		if (s3c_rtc_cpu_type == TYPE_S3C64XX)
			tmp &= ~S3C64XX_RTCCON_TICEN;
		tmp &= ~S3C2410_RTCCON_RTCEN;
		writew(tmp, base + S3C2410_RTCCON);

		if (s3c_rtc_cpu_type == TYPE_S3C2410) {
			tmp = readb(base + S3C2410_TICNT);
			tmp &= ~S3C2410_TICNT_ENABLE;
			writeb(tmp, base + S3C2410_TICNT);
		}
	} else {
		/* re-enable the device, and check it is ok */

		if ((readw(base+S3C2410_RTCCON) & S3C2410_RTCCON_RTCEN) == 0) {
			dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp | S3C2410_RTCCON_RTCEN,
				base + S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)) {
			dev_info(&pdev->dev, "removing RTCCON_CNTSEL\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp & ~S3C2410_RTCCON_CNTSEL,
				base + S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)) {
			dev_info(&pdev->dev, "removing RTCCON_CLKRST\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp & ~S3C2410_RTCCON_CLKRST,
				base + S3C2410_RTCCON);
		}
	}
}

static int __devexit s3c_rtc_remove(struct platform_device *dev)
{
	struct rtc_device *rtc = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	rtc_device_unregister(rtc);

	s3c_rtc_setaie(&dev->dev, 0);

	clk_disable(rtc_clk);
	clk_put(rtc_clk);
	rtc_clk = NULL;

	iounmap(s3c_rtc_base);
	release_resource(s3c_rtc_mem);
	kfree(s3c_rtc_mem);

	return 0;
}

/**
 * [1]P207详细说明该函数.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int __devinit s3c_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct rtc_time rtc_tm;
	struct resource *res;
	int ret;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	/* find the IRQs */
	/* 以下几行获得RTC实时时钟的中断[1]P207. */
	s3c_rtc_tickno = platform_get_irq(pdev, 1);
	if (s3c_rtc_tickno < 0) {
		dev_err(&pdev->dev, "no irq for rtc tick\n");
		return -ENOENT;
	}

	s3c_rtc_alarmno = platform_get_irq(pdev, 0);
	if (s3c_rtc_alarmno < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	pr_debug("s3c2410_rtc: tick irq %d, alarm irq %d\n",
		 s3c_rtc_tickno, s3c_rtc_alarmno);

	/* get the memory region */
	/* 以下几行获得RTC的IO内存映射[1]P207. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	s3c_rtc_mem = request_mem_region(res->start,
					 res->end-res->start+1,
					 pdev->name);

	if (s3c_rtc_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}

	s3c_rtc_base = ioremap(res->start, res->end - res->start + 1);
	if (s3c_rtc_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	rtc_clk = clk_get(&pdev->dev, "rtc");
	if (IS_ERR(rtc_clk)) {
		dev_err(&pdev->dev, "failed to find rtc clock source\n");
		ret = PTR_ERR(rtc_clk);
		rtc_clk = NULL;
		goto err_clk;
	}

	clk_enable(rtc_clk);

	/* check to see if everything is setup correctly */

	s3c_rtc_enable(pdev, 1);

	pr_debug("s3c2410_rtc: RTCCON=%02x\n",
		 readw(s3c_rtc_base + S3C2410_RTCCON));

	device_init_wakeup(&pdev->dev, 1);

	/* register RTC and exit */

	rtc = rtc_device_register("s3c", &pdev->dev, &s3c_rtcops,
				  THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	s3c_rtc_cpu_type = platform_get_device_id(pdev)->driver_data;

	/* Check RTC Time */

	s3c_rtc_gettime(NULL, &rtc_tm);

	if (rtc_valid_tm(&rtc_tm)) {
		rtc_tm.tm_year	= 100;
		rtc_tm.tm_mon	= 0;
		rtc_tm.tm_mday	= 1;
		rtc_tm.tm_hour	= 0;
		rtc_tm.tm_min	= 0;
		rtc_tm.tm_sec	= 0;

		s3c_rtc_settime(NULL, &rtc_tm);

		dev_warn(&pdev->dev, "warning: invalid RTC value so initializing it\n");
	}

	if (s3c_rtc_cpu_type == TYPE_S3C64XX)
		rtc->max_user_freq = 32768;
	else
		rtc->max_user_freq = 128;

	platform_set_drvdata(pdev, rtc);

	s3c_rtc_setfreq(&pdev->dev, 1);

	return 0;

 err_nortc:
	s3c_rtc_enable(pdev, 0);
	clk_disable(rtc_clk);
	clk_put(rtc_clk);

 err_clk:
	iounmap(s3c_rtc_base);

 err_nomap:
	release_resource(s3c_rtc_mem);

 err_nores:
	return ret;
}

#ifdef CONFIG_PM

/* RTC Power management control */

static int ticnt_save, ticnt_en_save;

static int s3c_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* save TICNT for anyone using periodic interrupts */
	ticnt_save = readb(s3c_rtc_base + S3C2410_TICNT);
	if (s3c_rtc_cpu_type == TYPE_S3C64XX) {
		ticnt_en_save = readw(s3c_rtc_base + S3C2410_RTCCON);
		ticnt_en_save &= S3C64XX_RTCCON_TICEN;
	}
	s3c_rtc_enable(pdev, 0);

	if (device_may_wakeup(&pdev->dev) && !wake_en) {
		if (enable_irq_wake(s3c_rtc_alarmno) == 0)
			wake_en = true;
		else
			dev_err(&pdev->dev, "enable_irq_wake failed\n");
	}

	return 0;
}

static int s3c_rtc_resume(struct platform_device *pdev)
{
	unsigned int tmp;

	s3c_rtc_enable(pdev, 1);
	writeb(ticnt_save, s3c_rtc_base + S3C2410_TICNT);
	if (s3c_rtc_cpu_type == TYPE_S3C64XX && ticnt_en_save) {
		tmp = readw(s3c_rtc_base + S3C2410_RTCCON);
		writew(tmp | ticnt_en_save, s3c_rtc_base + S3C2410_RTCCON);
	}

	if (device_may_wakeup(&pdev->dev) && wake_en) {
		disable_irq_wake(s3c_rtc_alarmno);
		wake_en = false;
	}

	return 0;
}
#else
#define s3c_rtc_suspend NULL
#define s3c_rtc_resume  NULL
#endif

static struct platform_device_id s3c_rtc_driver_ids[] = {
	{
		.name		= "s3c2410-rtc",
		.driver_data	= TYPE_S3C2410,
	}, {
		.name		= "s3c64xx-rtc",
		.driver_data	= TYPE_S3C64XX,
	},
	{ }
};

MODULE_DEVICE_TABLE(platform, s3c_rtc_driver_ids);

/**
 * RTC平台驱动设备[1]P206.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static struct platform_driver s3c_rtc_driver = {
	.probe		= s3c_rtc_probe, 				/* RTC探测函数[1]P206. */
	.remove		= __devexit_p(s3c_rtc_remove), 	/* RTC移除函数[1]P206. */
	.suspend	= s3c_rtc_suspend, 				/* RTC挂起函数[1]P206. */
	.resume		= s3c_rtc_resume, 				/* RTC恢复函数[1]P206. */
	.id_table	= s3c_rtc_driver_ids, 			
	.driver		= { 							/* 内嵌驱动程序结构[1]P206. */
		.name	= "s3c-rtc", 					/* 驱动名字[1]P206. */
		.owner	= THIS_MODULE, 					/* 驱动的模块[1]P206. */
	},
};

/** RTC标识语[1]P205
 *
 * @see Linux驱动开发入门与实战(2)
 */
static char __initdata banner[] = "S3C24XX RTC, (c) 2004,2006 Simtec Electronics\n";

/**
 * RTC驱动加载函数[1]P205.
 *
 * 先注册一个平台设备驱动, 然后由平台设备驱动负责完成对
 * RTC的驱动工作[1]P205.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int __init s3c_rtc_init(void)
{
	printk(banner); 		/* 打印RTC标识语[1]P206. */
	return platform_driver_register(&s3c_rtc_driver); /* 注册平台设备驱动[1]P206. */
}


/**
 * RTC卸载函数[1]P205.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void __exit s3c_rtc_exit(void)
{
	platform_driver_unregister(&s3c_rtc_driver); /* 卸载平台设备驱动[1]P206. */
}

module_init(s3c_rtc_init);
module_exit(s3c_rtc_exit);

MODULE_DESCRIPTION("Samsung S3C RTC Driver");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c2410-rtc");
