/* linux/arch/arm/plat-samsung/dev-wdt.c
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C series device definition for the watchdog timer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/devs.h>

/**
 * 该变量是s3c2440的看门狗资源[1]P227.
 * 
 * s3c2440只使用了IO内存和IRQ资源[1]P227.
 *
 * @see Linux驱动开发入门与实战(2)
 */
static struct resource s3c_wdt_resource[] = {
	[0] = {
		.start	= S3C_PA_WDT, 			/* 看门口IO内存开始位置, 被定义为WTCON的地址0x5300000[1]P227 */
		.end	= S3C_PA_WDT + SZ_1K, 	/* 1M的地址空间[1]P227. */
		.flags	= IORESOURCE_MEM, 		/* IO内存资源[1]P227. */
	},
	[1] = {
		.start	= IRQ_WDT, 				/* 看门狗的开始中断号, 被定义为80[1]227. */
		.end	= IRQ_WDT, 				/* 看门狗的开始中断号[1]P227. */
		.flags	= IORESOURCE_IRQ, 		/* 中断的IRQ资源[1]P227. */
	}
};

/**
 * [1]P226-227说明.
 *  
 * @see Linux驱动开发入门与实战(2)
 */
struct platform_device s3c_device_wdt = {
	.name		= "s3c2410-wdt", 				/* 平台设备名称[1]P227. */
	.id		= -1, 								/* 一般设为-1[1]P227. */
	.num_resources	= ARRAY_SIZE(s3c_wdt_resource), /* 资源数量[1]P227. */
	.resource	= s3c_wdt_resource, 				/* 资源的指针[1]P227. */
};
EXPORT_SYMBOL(s3c_device_wdt);
