/* arch/arm/mach-s3c2410/include/mach/fb.h
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 * Inspired by pxafb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_FB_H
#define __ASM_ARM_FB_H

#include <mach/regs-lcd.h>

/**
 * 该结构体对应LCD设备的寄存器, 通过该结构体可以映射到LCD的5个配置寄存器[1]P289.
 *
 * @see Linux驱动开发入门与实战(2)
 */
struct s3c2410fb_hw {
	unsigned long	lcdcon1; 	/* 5个LCD配置寄存器[1]P289. */
	unsigned long	lcdcon2;
	unsigned long	lcdcon3;
	unsigned long	lcdcon4;
	unsigned long	lcdcon5;
};

/** 
 * LCD设备的机器信息, 例如LCD显示器的宽度,高度
 * 和每个像素占多少位等信息[1]P288.
 *
 * LCD description 
 *
 * @see Linux驱动开发入门与实战(2)
 */
struct s3c2410fb_display {
	/* LCD type */
	unsigned type; 		/*!< LCD显示屏的类型[1]P289. */

	/* Screen size */
	unsigned short width; 	/*!< 屏幕大小, 这里是宽度[1]P289. */
	unsigned short height; 	/*!< 屏幕大小, 这里是高度[1]P289. */

	/* Screen info */
	unsigned short xres; 	/*!< 以下3行存储的是屏幕信息[1]P289. */
	unsigned short yres;
	unsigned short bpp;

	unsigned pixclock;		/* pixclock in picoseconds */
	unsigned short left_margin;  /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short right_margin; /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short hsync_len;    /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short upper_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short lower_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short vsync_len;	/* value in lines (TFT) or 0 (STN) */

	/* lcd configuration registers */
	unsigned long	lcdcon5;  /*!< LCD配置寄存器[1]P289. */
};

/**
 * 表示LCD显示器的平台信息[1]P288. 
 *
 * @see Linux驱动开发入门与实战(2)
 */
struct s3c2410fb_mach_info {
	struct s3c2410fb_display *displays;	/*!< attached diplays info; 存储相似信息[1]P288. */
	unsigned num_displays;			/*!< number of defined displays; 显示缓冲的数量[1]P288. */
	unsigned default_display;

	/* GPIOs; GPIO引脚[1]P288. */
	unsigned long	gpcup;
	unsigned long	gpcup_mask;
	unsigned long	gpccon;
	unsigned long	gpccon_mask;
	unsigned long	gpdup;
	unsigned long	gpdup_mask;
	unsigned long	gpdcon;
	unsigned long	gpdcon_mask;

	/* lpc3600 control register */
	unsigned long	lpcsel;
};

extern void __init s3c24xx_fb_set_platdata(struct s3c2410fb_mach_info *);

#endif /* __ASM_ARM_FB_H */
