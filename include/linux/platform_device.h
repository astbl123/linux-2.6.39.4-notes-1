/*
 * platform_device.h - generic, centralized driver model
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 *
 * This file is released under the GPLv2
 *
 * See Documentation/driver-model/ for more information.
 */

#ifndef _PLATFORM_DEVICE_H_
#define _PLATFORM_DEVICE_H_

#include <linux/device.h>
#include <linux/mod_devicetable.h>

struct mfd_cell;

/**
 * Linux2.6开始引入了一套新的驱动管理和注册模型: 平台设备(platform device)和
 * 平台驱动(platform driver)[1]P226.
 * 
 * Linux中大部分设备驱动都可以使用这套机制, 设备使用platform_device表示,
 * 驱动使用platform_driver表示[1]P226.
 *
 * 平台设备是指处理器上集成的额外功能的附加设备, 如watchdog, IIC, IIS, RTC 
 * 和ADC等设备[1]P226.
 *
 * 为了便于统一管理平台设备的资源, 该结构体定义了平台设备所使用的资源. 
 * 这些资源都与特定处理器相关, 需要驱动开发者查阅相关的处理器数据手册来
 * 编写[1]P227.
 * 
 * @see Linux驱动开发入门与实战(2)
 */
struct platform_device {
	const char	* name; 	/*!< 平台设备的名字, 与驱动的名字对应[1]P226. */
	int		id;  			/*!< 与驱动绑定有关, 一般为-1[1]P226. */
	struct device	dev; 	/*!< 设备结构体, 说明platform_device派生于device[1]P226. */
	u32		num_resources;  /*!< 设备使用的资源数量[1]P226. */
	struct resource	* resource; /*!< 指向资源的数组, 数量由num_resources指定[1]P226. */

	const struct platform_device_id	*id_entry;

	/* MFD cell pointer */
	struct mfd_cell *mfd_cell;

	/* arch specific additions */
	struct pdev_archdata	archdata;
};

#define platform_get_device_id(pdev)	((pdev)->id_entry)

/**
 * 从device结构转到平台设备[1]P216.
 *
 * @see Linu驱动开发入门与实战(2)
 */
#define to_platform_device(x) container_of((x), struct platform_device, dev)

extern int platform_device_register(struct platform_device *);
extern void platform_device_unregister(struct platform_device *);

extern struct bus_type platform_bus_type;
extern struct device platform_bus;

extern struct resource *platform_get_resource(struct platform_device *, unsigned int, unsigned int);
extern int platform_get_irq(struct platform_device *, unsigned int);
extern struct resource *platform_get_resource_byname(struct platform_device *, unsigned int, const char *);
extern int platform_get_irq_byname(struct platform_device *, const char *);
extern int platform_add_devices(struct platform_device **, int);

extern struct platform_device *platform_device_register_resndata(
		struct device *parent, const char *name, int id,
		const struct resource *res, unsigned int num,
		const void *data, size_t size);

/**
 * platform_device_register_simple - add a platform-level device and its resources
 * @name: base name of the device we're adding
 * @id: instance id
 * @res: set of resources that needs to be allocated for the device
 * @num: number of resources
 *
 * This function creates a simple platform device that requires minimal
 * resource and memory management. Canned release function freeing memory
 * allocated for the device allows drivers using such devices to be
 * unloaded without waiting for the last reference to the device to be
 * dropped.
 *
 * This interface is primarily intended for use with legacy drivers which
 * probe hardware directly.  Because such drivers create sysfs device nodes
 * themselves, rather than letting system infrastructure handle such device
 * enumeration tasks, they don't fully conform to the Linux driver model.
 * In particular, when such drivers are built as modules, they can't be
 * "hotplugged".
 *
 * Returns &struct platform_device pointer on success, or ERR_PTR() on error.
 */
static inline struct platform_device *platform_device_register_simple(
		const char *name, int id,
		const struct resource *res, unsigned int num)
{
	return platform_device_register_resndata(NULL, name, id,
			res, num, NULL, 0);
}

/**
 * platform_device_register_data - add a platform-level device with platform-specific data
 * @parent: parent device for the device we're adding
 * @name: base name of the device we're adding
 * @id: instance id
 * @data: platform specific data for this platform device
 * @size: size of platform specific data
 *
 * This function creates a simple platform device that requires minimal
 * resource and memory management. Canned release function freeing memory
 * allocated for the device allows drivers using such devices to be
 * unloaded without waiting for the last reference to the device to be
 * dropped.
 *
 * Returns &struct platform_device pointer on success, or ERR_PTR() on error.
 */
static inline struct platform_device *platform_device_register_data(
		struct device *parent, const char *name, int id,
		const void *data, size_t size)
{
	return platform_device_register_resndata(parent, name, id,
			NULL, 0, data, size);
}

extern struct platform_device *platform_device_alloc(const char *name, int id);
extern int platform_device_add_resources(struct platform_device *pdev,
					 const struct resource *res,
					 unsigned int num);
extern int platform_device_add_data(struct platform_device *pdev, const void *data, size_t size);
extern int platform_device_add(struct platform_device *pdev);
extern void platform_device_del(struct platform_device *pdev);
extern void platform_device_put(struct platform_device *pdev);

/**
 * [1]P206详细说明了各个函数的作用.
 *
 * 每一个平台设备(platform_device)都对应一个平台设备驱动,
 * 这个驱动用来对平台设备进行探测, 移除, 关闭和电源管理[1]P228.
 * 
 * match(): 
 * 		内核启动时, 会注册平台设备和平台设备驱动程序, 并且在适当的时候
 * 	将平台设备和平台设备驱动程序连接起来, 方法就是用系统中的所有平
 * 	台设备和已经注册的平台驱动进行匹配, 匹配就由该函数实现[1]P229.
 * 		该函数比较平台设备的name字段和驱动的name字段, 相同时, 表示
 * 	匹配成功, 返回1; 不同时,表示匹配失败, 返回0[1]P229;
 * 		该函数由于内核自己调用[1]P229.
 * 
 * probe():
 * 		当设备找到对应的驱动时, 会触发probe()函数, 所以probe()函数一般是驱动
 * 	加载成功后调用的第一个函数, 在该函数中可以申请设备所需要的资源[1]P229.
 * 
 * remove():
 * 		当设备可以移除, 为了减少所占用的系统资源, 那么就应该实现该函数. 该函
 * 	数一般于probe()对应, 在probe()中申请的资源, 应该在该函数中释放[1]P229.
 *
 * 	shutdown():
 * 		当设备断电或者关闭时被调用[1]P229.
 *
 * 	suspend():
 * 		使设备处于低功耗状态[1]P229.
 *
 * 	resume():
 * 		使设备从低功耗状态恢复[1]P229.
 * 
 * @see Linux驱动开发入门与实战(2)
 */
struct platform_driver {
	int (*probe)(struct platform_device *);  	/*!< 当设备与启动匹配后执行该函数, 在其中分配资源, 是加载模块后执行的第1个函数[1]P206. */
	int (*remove)(struct platform_device *); 	/*!< 与probe()相反, 在其中释放资源[1]P206. */
	void (*shutdown)(struct platform_device *); /*!<  */
	int (*suspend)(struct platform_device *, pm_message_t state); /*!< 该函数使设备处于低功耗状态[1]P206. */
	int (*resume)(struct platform_device *); 					  /*!< 与suspend()相反, 使设备从低功耗状态恢复[1]P206. */
	struct device_driver driver; 			    /*!< 设备驱动模型中定义的驱动结构体[1]P206. */
	const struct platform_device_id *id_table;  /*!< */
};

extern int platform_driver_register(struct platform_driver *);
extern void platform_driver_unregister(struct platform_driver *);

/* non-hotpluggable platform devices may use this so that probe() and
 * its support may live in __init sections, conserving runtime memory.
 */
extern int platform_driver_probe(struct platform_driver *driver,
		int (*probe)(struct platform_device *));

/**
 * 从pdev->dev的私有数据中得到rtc_device结构[1]P216.
 *
 * 从platform_device中获得fb_info信息[2]P145.
 *
 * @see [1]Linux驱动开发入门与实战(2)
 * @see [2]Linux系统移植(2)
 */
static inline void *platform_get_drvdata(const struct platform_device *pdev)
{
	return dev_get_drvdata(&pdev->dev);
}

static inline void platform_set_drvdata(struct platform_device *pdev, void *data)
{
	dev_set_drvdata(&pdev->dev, data);
}

extern struct platform_device *platform_create_bundle(struct platform_driver *driver,
					int (*probe)(struct platform_device *),
					struct resource *res, unsigned int n_res,
					const void *data, size_t size);

extern const struct dev_pm_ops * platform_bus_get_pm_ops(void);
extern void platform_bus_set_pm_ops(const struct dev_pm_ops *pm);

/* early platform driver interface */
struct early_platform_driver {
	const char *class_str;
	struct platform_driver *pdrv;
	struct list_head list;
	int requested_id;
	char *buffer;
	int bufsize;
};

#define EARLY_PLATFORM_ID_UNSET -2
#define EARLY_PLATFORM_ID_ERROR -3

extern int early_platform_driver_register(struct early_platform_driver *epdrv,
					  char *buf);
extern void early_platform_add_devices(struct platform_device **devs, int num);

static inline int is_early_platform_device(struct platform_device *pdev)
{
	return !pdev->dev.driver;
}

extern void early_platform_driver_register_all(char *class_str);
extern int early_platform_driver_probe(char *class_str,
				       int nr_probe, int user_only);
extern void early_platform_cleanup(void);

#define early_platform_init(class_string, platdrv)		\
	early_platform_init_buffer(class_string, platdrv, NULL, 0)

#ifndef MODULE
#define early_platform_init_buffer(class_string, platdrv, buf, bufsiz)	\
static __initdata struct early_platform_driver early_driver = {		\
	.class_str = class_string,					\
	.buffer = buf,							\
	.bufsize = bufsiz,						\
	.pdrv = platdrv,						\
	.requested_id = EARLY_PLATFORM_ID_UNSET,			\
};									\
static int __init early_platform_driver_setup_func(char *buffer)	\
{									\
	return early_platform_driver_register(&early_driver, buffer);	\
}									\
early_param(class_string, early_platform_driver_setup_func)
#else /* MODULE */
#define early_platform_init_buffer(class_string, platdrv, buf, bufsiz)	\
static inline char *early_platform_driver_setup_func(void)		\
{									\
	return bufsiz ? buf : NULL;					\
}
#endif /* MODULE */

#endif /* _PLATFORM_DEVICE_H_ */
