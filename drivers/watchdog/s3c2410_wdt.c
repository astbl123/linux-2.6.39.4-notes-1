/* linux/drivers/char/watchdog/s3c2410_wdt.c
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Watchdog Timer Support
 *
 * Based on, softdog.c by Alan Cox,
 *     (c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>

#include <mach/map.h>

#undef S3C_VA_WATCHDOG
#define S3C_VA_WATCHDOG (0)

#include <plat/regs-watchdog.h>

#define PFX "s3c2410-wdt: "

#define CONFIG_S3C2410_WATCHDOG_ATBOOT		(0)
#define CONFIG_S3C2410_WATCHDOG_DEFAULT_TIME	(15)

/*
 *@see [1]Linux驱动开发入门与实战(2)
 */
static int nowayout	= WATCHDOG_NOWAYOUT; 	/* 表示决不允许看门狗关闭[1]P232. */ 
static int tmr_margin	= CONFIG_S3C2410_WATCHDOG_DEFAULT_TIME; /* 默认看门狗喂狗时间为15秒[1]P232. */
static int tmr_atboot	= CONFIG_S3C2410_WATCHDOG_ATBOOT; 		/* 表示系统启动时就使能看门狗, 1表示使能, 0表示关闭[1]P232, */
static int soft_noboot;  /* 表示看门狗的工作方式, 看门狗可以作为定时器使用, 也可以作为复位硬件使用. 该变量1表示作为定时器使用[1]P232. */
static int debug; 		 /* 表示是否使用调试模式来调试代码, 该模式中, 会打印调试信息[1]P232. */

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,    int, 0);
module_param(soft_noboot, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default="
		__MODULE_STRING(CONFIG_S3C2410_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(tmr_atboot,
		"Watchdog is started at boot time if set to 1, default="
			__MODULE_STRING(CONFIG_S3C2410_WATCHDOG_ATBOOT));
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, "
			"0 to reboot (default 0)");
MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug (default 0)");

static unsigned long open_lock;
static struct device    *wdt_dev;	/* platform device attached to */
static struct resource	*wdt_mem;
static struct resource	*wdt_irq;
static struct clk	*wdt_clock;
static void __iomem	*wdt_base;
static unsigned int	 wdt_count;
static char		 expect_close;
static DEFINE_SPINLOCK(wdt_lock);

/* watchdog control routines */

#define DBG(msg...) do { \
	if (debug) \
		printk(KERN_INFO msg); \
	} while (0)

/* functions */

/**
 * 该函数重新装载了WTCNT寄存器, 相当于一个喂狗的功能[1]P239.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c2410wdt_keepalive(void)
{
	spin_lock(&wdt_lock); 			/* 获得自旋锁[1]P240. */
	writel(wdt_count, wdt_base + S3C2410_WTCNT); /* 重写计数寄存器WTCNT[1]P240. */
	spin_unlock(&wdt_lock); 		/* 释放自旋锁[1]P240. */
}

/**
 * 该函数完成实际的看门狗停止工作[1]P236.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void __s3c2410wdt_stop(void)
{
	unsigned long wtcon;

	wtcon = readl(wdt_base + S3C2410_WTCON);
	wtcon &= ~(S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN);
	writel(wtcon, wdt_base + S3C2410_WTCON);
}

/**
 * 停止看门狗[1]P236.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c2410wdt_stop(void)
{
	spin_lock(&wdt_lock);
	__s3c2410wdt_stop();
	spin_unlock(&wdt_lock);
}

/**
 * 开始看门狗[1]P236.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c2410wdt_start(void)
{
	unsigned long wtcon; 			/* 暂时存储WTCNT的值[1]P236. */

	spin_lock(&wdt_lock); 			/* 避免不同线程同时访问临界资源[1]P236. */

	__s3c2410wdt_stop(); 			/* 先停止看门狗便于设置[1]P236. */

	wtcon = readl(wdt_base + S3C2410_WTCON); /* 读出WTCON的值[1]P236. */
	wtcon |= S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV128; /* 通过设置WTCON的第5位允许看门狗, 并将第3,4位设为11, 使用四分频[1]P236. */

	if (soft_noboot) { 					/* 将看门狗当做定时器用[1]P236. */
		wtcon |= S3C2410_WTCON_INTEN; 	/* 使能中断[1]P236. */
		wtcon &= ~S3C2410_WTCON_RSTEN; 	/* 不允许发出复位信号[1]P236. */
	} else { 							/* 看门狗当做复位器使用[1]P236. */
		wtcon &= ~S3C2410_WTCON_INTEN; 	/* 禁止发出中断[1]P236. */
		wtcon |= S3C2410_WTCON_RSTEN; 	/* 允许发出复位信号[1]P236. */
	}

	DBG("%s: wdt_count=0x%08x, wtcon=%08lx\n", 	/* 打印相关信息用于调试[1]P236. */
	    __func__, wdt_count, wtcon);

	writel(wdt_count, wdt_base + S3C2410_WTDAT); /* 重写数据寄存器的值[1]P236. */
	writel(wdt_count, wdt_base + S3C2410_WTCNT); /* 重新写计数寄存器的值[1]P236, */
	writel(wtcon, wdt_base + S3C2410_WTCON); 	 /* 写控制寄存器的值[1]P236. */
	spin_unlock(&wdt_lock); 	/* 自旋锁解锁[1]P236, */
}

static inline int s3c2410wdt_is_running(void)
{
	return readl(wdt_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE;
}

/**
 * 设置看门狗复位时间[1]P234.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c2410wdt_set_heartbeat(int timeout)
{
	unsigned long freq = clk_get_rate(wdt_clock); /* 获得看门狗时钟频率PCLK[1]P235. */
	unsigned int count; 						  /* 将填入WTCNT的计数值[1]P235. */
	unsigned int divisor = 1; 					  /* 将填入WTCON[15:8]的预分频系数[1]P235. */
	unsigned long wtcon; 						  /* 暂时存储WTCON的值[1]P235. */

	if (timeout < 1) 							  /* 看门狗的复位时间不能小于1秒[1]P235. */
		return -EINVAL;

	freq /= 128; 								  /* 看门狗默认使用128的四相分频[1]P235. */
	count = timeout * freq; 					  /* 秒数乘以每秒的时钟滴答等于计数值[1]P235. */

	DBG("%s: count=%d, timeout=%d, freq=%lu\n", 
	    __func__, count, timeout, freq);

	/* if the count is bigger than the watchdog register,
	   then work out what we need to do (and if) we can
	   actually make this value
	* 
	* 最终填入的计数值不能大于WTCNT的范围, WTCNT是一个16位寄存器, 其最大值为0x10000[1]P235.
	*/
	if (count >= 0x10000) { 	
		for (divisor = 1; divisor <= 0x100; divisor++) { /* 从1~256, 寻找一个适合的预分频系数[1]P235. */
			if ((count / divisor) < 0x10000)
				break; 									 /* 找到则退出[1]P235. */
		}

		/* 经过预分频和4相分配的计数值仍大于0x10000, 则
		 * 复位时间太长, 看门狗不支持[1]P235. 		 */
		if ((count / divisor) >= 0x10000) {
			dev_err(wdt_dev, "timeout %d too big\n", timeout);
			return -EINVAL;
		}
	}

	tmr_margin = timeout; 	/* 合法的复位时间[1]P235. */

	DBG("%s: timeout=%d, divisor=%d, count=%d (%08x)\n",
	    __func__, timeout, divisor, count, count/divisor);

	count /= divisor; 		/* 分频后最终得到计数值[1]P235. */
	wdt_count = count;

	/* update the pre-scaler */
	wtcon = readl(wdt_base + S3C2410_WTCON); /* 读取WTCNT的值[1]P235. */
	wtcon &= ~S3C2410_WTCON_PRESCALE_MASK; 	 /* 将WTCNT的高8位清零[1]P235. */
	wtcon |= S3C2410_WTCON_PRESCALE(divisor-1); /* 填入预分频系数[1]P235.*/

	writel(count, wdt_base + S3C2410_WTDAT); /* 将计数值写到数据寄存器WTDAT中[1]P235. */
	writel(wtcon, wdt_base + S3C2410_WTCON); /* 设置控制寄存器WTCON */

	return 0; 								 /* 成功返回0[1]P235. */
}

/**
 * 当用户程序调用open()函数时, 内核最终会调用该函数[1]P238.
 *  
 *	/dev/watchdog handling
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c2410wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &open_lock)) /* 只允许打开一次[1]P239. */
		return -EBUSY;

	if (nowayout) 	/* nowayout不为0, 表示看门狗设备绝对不允许关闭[1]P239. */
		__module_get(THIS_MODULE);

	expect_close = 0;

	/* start the timer */
	s3c2410wdt_start(); 	/* 开始运行看门狗设备[1]P239. */
	return nonseekable_open(inode, file); /* 不允许调用seek()[1]P239. */
}

/**
 * 为了使看门狗设备在调用close()函数关闭后, 能够使用open()方法重新打开,
 * 驱动需要定义该函数[1]P239.
 *
 * 应该在该函数中清除open_lock的第0位, 使设备能够被open()函数打开[1]P239.
 *
 * 如果看门狗允许关闭, 则应该调用s3c2410wdt_stop()函数关闭看门狗; 如果不允许
 * 关闭设备, 则调用s3c2410wdt_keepalive()函数使看门狗为活动状态[1]P239.
 *
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c2410wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *	Shut off the timer.
	 *	Lock it in if it's a module and we set nowayout
	 */

	if (expect_close == 42) 	/* 看门狗为允许状态[1]P239. */
		s3c2410wdt_stop(); 		/* 关闭看门狗[1]P239. */
	else {
		dev_err(wdt_dev, "Unexpected close, not stopping watchdog\n");
		s3c2410wdt_keepalive();
	}
	expect_close = 0; 			/* 设为不允许关闭[1]P239. */
	clear_bit(0, &open_lock); 	/* 将open_lock的第0位设为0, 是原子操作[1]P239. */
	return 0;
}

/**
 * 该函数主要用于用来设置expect_close变量为允许关闭状态, 如果向
 * 看门狗设备中写入V, 那么就允许关闭设备[1]P240.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static ssize_t s3c2410wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	/*
	 *	Refresh the timer.
	 */
	if (len) { /* 有数据写入len不为0[1]P240. */
		if (!nowayout) { 	/* 允许关闭[1]P240. */
			size_t i;

			/* In case it was set long ago */
			expect_close = 0; 	/* 允许关闭状态[1]P240. */

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
		s3c2410wdt_keepalive(); /* 允许关闭[1]P240. */
	}
	return len;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info s3c2410_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"S3C2410 Watchdog",
};


/**
 * 该函数用于接受一些系统命令, 用来设置看门狗的内部状态[1]P240.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static long s3c2410wdt_ioctl(struct file *file,	unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_margin;

	switch (cmd) {
	case WDIOC_GETSUPPORT: /* 获得看门狗设备的信息[1]P240. */
		return copy_to_user(argp, &s3c2410_wdt_ident,
			sizeof(s3c2410_wdt_ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:  /* 获得看门狗状态[1]P240. */
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:  /* 对看门狗进行喂狗操作[1]P240. */
		s3c2410wdt_keepalive();
		return 0;
	case WDIOC_SETTIMEOUT: /* 设置新的超时时间[1]P240. */
		if (get_user(new_margin, p)) /* 从用户空间获得超时时间[1]P241. */
			return -EFAULT;
		if (s3c2410wdt_set_heartbeat(new_margin))
			return -EINVAL;
		s3c2410wdt_keepalive(); /* 喂狗操作[1]P241. */
		return put_user(tmr_margin, p); /* 返回旧的超时时间[1]P241. */
	case WDIOC_GETTIMEOUT: /* 获得看门狗设备当前超时时间[1]P241. */
		return put_user(tmr_margin, p);
	default:
		return -ENOTTY;
	}
}

/* kernel interface */

static const struct file_operations s3c2410wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= s3c2410wdt_write,
	.unlocked_ioctl	= s3c2410wdt_ioctl,
	.open		= s3c2410wdt_open,
	.release	= s3c2410wdt_release,
};

static struct miscdevice s3c2410wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &s3c2410wdt_fops,
};

/**
 * 当看门狗作为定时器使用时, 发出中断信号而不发出复位信号.
 * 该中断在s3c2410wdt_probe()中通过调用request_irq()函数向
 * 系统做了申请. 该中断处理函数的主要功能是喂狗操作, 使看
 * 门狗重新开始计数[1]P241.
 *
 * interrupt handler code 
 *
 * @see Linux驱动开发入门与实战(2)
 */
static irqreturn_t s3c2410wdt_irq(int irqno, void *param)
{
	dev_info(wdt_dev, "watchdog timer expired (irq)\n"); /* 调试信息[1]P241. */

	s3c2410wdt_keepalive(); /* 看门狗喂狗操作[1]P241. */
	return IRQ_HANDLED;
}

#ifdef CONFIG_CPU_FREQ

static int s3c2410wdt_cpufreq_transition(struct notifier_block *nb,
					  unsigned long val, void *data)
{
	int ret;

	if (!s3c2410wdt_is_running())
		goto done;

	if (val == CPUFREQ_PRECHANGE) {
		/* To ensure that over the change we don't cause the
		 * watchdog to trigger, we perform an keep-alive if
		 * the watchdog is running.
		 */

		s3c2410wdt_keepalive();
	} else if (val == CPUFREQ_POSTCHANGE) {
		s3c2410wdt_stop();

		ret = s3c2410wdt_set_heartbeat(tmr_margin);

		if (ret >= 0)
			s3c2410wdt_start();
		else
			goto err;
	}

done:
	return 0;

 err:
	dev_err(wdt_dev, "cannot set new value for timeout %d\n", tmr_margin);
	return ret;
}

static struct notifier_block s3c2410wdt_cpufreq_transition_nb = {
	.notifier_call	= s3c2410wdt_cpufreq_transition,
};

static inline int s3c2410wdt_cpufreq_register(void)
{
	return cpufreq_register_notifier(&s3c2410wdt_cpufreq_transition_nb,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void s3c2410wdt_cpufreq_deregister(void)
{
	cpufreq_unregister_notifier(&s3c2410wdt_cpufreq_transition_nb,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else
static inline int s3c2410wdt_cpufreq_register(void)
{
	return 0;
}

static inline void s3c2410wdt_cpufreq_deregister(void)
{
}
#endif



/** 
 * match()成功后, 执行该函数[1]P233.
 *
 * device interface 
 * 
 * @see Linux驱动开发入门与实战(2)
 */
static int __devinit s3c2410wdt_probe(struct platform_device *pdev)
{
	struct device *dev; 			/* 设备结构指针[1]P233. */
	unsigned int wtcon; 			/* 用于暂时存放WTCON寄存器的数据[1]P233. */
	int started = 0;
	int ret;
	int size;

	DBG("%s: probe=%p\n", __func__, pdev);

	dev = &pdev->dev;
	wdt_dev = &pdev->dev; 			/* 平台设备的设备结构体device[1]P233. */

	/* get the memory region for the watchdog timer */

	wdt_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (wdt_mem == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = resource_size(wdt_mem);
	if (!request_mem_region(wdt_mem->start, size, pdev->name)) {
		dev_err(dev, "failed to get memory region\n");
		return -EBUSY;
	}

	wdt_base = ioremap(wdt_mem->start, size);
	if (wdt_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_req;
	}

	DBG("probe: mapped wdt_base=%p\n", wdt_base);

	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0); /* 获得看门狗可以申请的中断号[1]P233. */
	if (wdt_irq == NULL) { 									  /* 获取中断号失败则退出[1]P233. */
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_map;
	}

	ret = request_irq(wdt_irq->start, s3c2410wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_map;
	}

	wdt_clock = clk_get(&pdev->dev, "watchdog");
	if (IS_ERR(wdt_clock)) {
		dev_err(dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(wdt_clock);
		goto err_irq;
	}

	clk_enable(wdt_clock);

	if (s3c2410wdt_cpufreq_register() < 0) {
		printk(KERN_ERR PFX "failed to register cpufreq\n");
		goto err_clk;
	}

	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */

	if (s3c2410wdt_set_heartbeat(tmr_margin)) {
		started = s3c2410wdt_set_heartbeat(
					CONFIG_S3C2410_WATCHDOG_DEFAULT_TIME);

		if (started == 0)
			dev_info(dev,
			   "tmr_margin value out of range, default %d used\n",
			       CONFIG_S3C2410_WATCHDOG_DEFAULT_TIME);
		else
			dev_info(dev, "default timer value is out of range, "
							"cannot start\n");
	}

	ret = misc_register(&s3c2410wdt_miscdev);
	if (ret) {
		dev_err(dev, "cannot register miscdev on minor=%d (%d)\n",
			WATCHDOG_MINOR, ret);
		goto err_cpufreq;
	}

	if (tmr_atboot && started == 0) {
		dev_info(dev, "starting watchdog timer\n");
		s3c2410wdt_start();
	} else if (!tmr_atboot) {
		/* if we're not enabling the watchdog, then ensure it is
		 * disabled if it has been left running from the bootloader
		 * or other source */

		s3c2410wdt_stop();
	}

	/* print out a statement of readiness */

	wtcon = readl(wdt_base + S3C2410_WTCON);

	dev_info(dev, "watchdog %sactive, reset %sabled, irq %sabled\n",
		 (wtcon & S3C2410_WTCON_ENABLE) ?  "" : "in",
		 (wtcon & S3C2410_WTCON_RSTEN) ? "" : "dis",
		 (wtcon & S3C2410_WTCON_INTEN) ? "" : "en");

	return 0;

 err_cpufreq:
	s3c2410wdt_cpufreq_deregister();

 err_clk:
	clk_disable(wdt_clock);
	clk_put(wdt_clock);

 err_irq:
	free_irq(wdt_irq->start, pdev);

 err_map:
	iounmap(wdt_base);

 err_req:
	release_mem_region(wdt_mem->start, size);
	wdt_mem = NULL;

	return ret;
}


/**
 * 看门狗驱动程序移除函数[1]P237.
 *
 *
 * @see Linux 驱动开发入门与实战(2)
 */
static int __devexit s3c2410wdt_remove(struct platform_device *dev)
{
	misc_deregister(&s3c2410wdt_miscdev);

	s3c2410wdt_cpufreq_deregister();

	clk_disable(wdt_clock);
	clk_put(wdt_clock);
	wdt_clock = NULL;

	free_irq(wdt_irq->start, dev);
	wdt_irq = NULL;

	iounmap(wdt_base);

	release_mem_region(wdt_mem->start, resource_size(wdt_mem));
	wdt_mem = NULL;
	return 0;
}

/**
 * 当看门狗被关闭时, 内核会自动调用该函数先停止看门狗设备[1]P237.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static void s3c2410wdt_shutdown(struct platform_device *dev)
{
	s3c2410wdt_stop();
}

#ifdef CONFIG_PM

static unsigned long wtcon_save;
static unsigned long wtdat_save;

/**
 * 暂停看门狗, 保存看门狗的寄存器, 并设置看门狗为停止状态[1]P237.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c2410wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Save watchdog state, and turn it off. */
	wtcon_save = readl(wdt_base + S3C2410_WTCON); /* 保存看门狗的当前状态 WTCON寄存器[1]P238. */
	wtdat_save = readl(wdt_base + S3C2410_WTDAT); /* 保存看门狗的当前状态 WTDAT寄存器[1]P238. */

	/* Note that WTCNT doesn't need to be saved. */
	s3c2410wdt_stop(); 	/* 停止看门狗[1]P238. */

	return 0;
}

/**
 * 与suspend()相反, 恢复看门狗寄存器的值[1]P238.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static int s3c2410wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */

	/* 恢复看门狗寄存器的值, 并重置WTCNT为WTDAT[1]P238. */
	writel(wtdat_save, wdt_base + S3C2410_WTDAT);
	writel(wtdat_save, wdt_base + S3C2410_WTCNT); /* Reset count */
	writel(wtcon_save, wdt_base + S3C2410_WTCON);

	printk(KERN_INFO PFX "watchdog %sabled\n", 	  /* 打印一些调试信息[1]P238. */
	       (wtcon_save & S3C2410_WTCON_ENABLE) ? "en" : "dis");

	return 0;
}

#else
#define s3c2410wdt_suspend NULL
#define s3c2410wdt_resume  NULL
#endif /* CONFIG_PM */

/**
 * s3c2410/s3c2440看门狗平台设备驱动[1]P228.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static struct platform_driver s3c2410wdt_driver = {
	.probe		= s3c2410wdt_probe,
	.remove		= __devexit_p(s3c2410wdt_remove),
	.shutdown	= s3c2410wdt_shutdown,
	.suspend	= s3c2410wdt_suspend,
	.resume		= s3c2410wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c2410-wdt",
	},
};


static char banner[] __initdata =
	KERN_INFO "S3C2410 Watchdog Timer, (c) 2004 Simtec Electronics\n";

/**
 * @see Linux驱动开发入门与实战(2)
 */
static int __init watchdog_init(void)
{
	printk(banner); /* 打印看门狗信息[1]P232. */
	return platform_driver_register(&s3c2410wdt_driver); /* 注册平台设备[1]P232. */
}

/**
 * @see Linux驱动开发入门与实战(2)
 */
static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&s3c2410wdt_driver); 	/* 注销平台设备[1]P232. */
} 

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>, "
	      "Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("S3C2410 Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:s3c2410-wdt");
