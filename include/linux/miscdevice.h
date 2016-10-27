#ifndef _LINUX_MISCDEVICE_H
#define _LINUX_MISCDEVICE_H
#include <linux/module.h>
#include <linux/major.h>

/*
 *	These allocations are managed by device@lanana.org. If you use an
 *	entry that is not in assigned your entry may well be moved and
 *	reassigned, or set dynamic if a fixed value is not justified.
 */

#define PSMOUSE_MINOR		1
#define MS_BUSMOUSE_MINOR	2
#define ATIXL_BUSMOUSE_MINOR	3
/*#define AMIGAMOUSE_MINOR	4	FIXME OBSOLETE */
#define ATARIMOUSE_MINOR	5
#define SUN_MOUSE_MINOR		6
#define APOLLO_MOUSE_MINOR	7
#define PC110PAD_MINOR		9
/*#define ADB_MOUSE_MINOR	10	FIXME OBSOLETE */
#define WATCHDOG_MINOR		130	/* Watchdog timer     */
#define TEMP_MINOR		131	/* Temperature Sensor */
#define RTC_MINOR		135
#define EFI_RTC_MINOR		136	/* EFI Time services */
#define SUN_OPENPROM_MINOR	139
#define DMAPI_MINOR		140	/* DMAPI */
#define NVRAM_MINOR		144
#define SGI_MMTIMER		153
#define STORE_QUEUE_MINOR	155
#define I2O_MINOR		166
#define MICROCODE_MINOR		184
#define TUN_MINOR		200
#define MWAVE_MINOR		219	/* ACP/Mwave Modem */
#define MPT_MINOR		220
#define MPT2SAS_MINOR		221
#define UINPUT_MINOR		223
#define HPET_MINOR		228
#define FUSE_MINOR		229
#define KVM_MINOR		232
#define BTRFS_MINOR		234
#define AUTOFS_MINOR		235
#define MAPPER_CTRL_MINOR	236
#define MISC_DYNAMIC_MINOR	255

struct device;

/**
 * 由于设备号比较紧张, 所以一些不相关的设备可以使用同一个主设备号,
 * 不同次设备号. 主设备号通常是10(网上有资料说, 混杂设备的主设备号
 * 固定为10[2].). 由于这个原因, 一些设备也可以叫做混杂设备, 使用该
 * 结构表示[1]P230.
 *
 * 混杂设备是一种特殊的字符设备, 所以其操作方法和字符设备的操作方法
 * 基本一样[1]P238.
 *
 * @see [1]Linux驱动开发入门与实战(2).
 * @see [2]Android驱动程序实例分析总结(http://wenku.baidu.com/link?url=Q-
 * pAWhelQhBxmda62lBOhbIwb4d8gOi00HXLeoJzkLO7dyjJZfoqhhQ4138xM-
 * 7JaglghivnYJPFDgbRbZwV0lV5Fb13NJOx8ekUN8YXA0y)
 */
struct miscdevice  {
	int minor; 							/*!< 次设备号[1]P230. */
	const char *name; 					/*!< 混杂设备名字[1]P230. */
	const struct file_operations *fops; /*!< 设备的操作函数, 与字符设备相同[1]P230. */ 
	struct list_head list; 				/*!< 指向下一个混杂设备的链表[1]P230. */
	struct device *parent; 				/*!< 指向父设备[1]P230. */
	struct device *this_device; 		/*!< 指向当前设备结构体[1]P230. */
	const char *nodename;
	mode_t mode;
};

extern int misc_register(struct miscdevice * misc);
extern int misc_deregister(struct miscdevice *misc);

#define MODULE_ALIAS_MISCDEV(minor)				\
	MODULE_ALIAS("char-major-" __stringify(MISC_MAJOR)	\
	"-" __stringify(minor))
#endif
