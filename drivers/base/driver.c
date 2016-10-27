/*
 * driver.c - centralized device driver management
 *
 * Copyright (c) 2002-3 Patrick Mochel
 * Copyright (c) 2002-3 Open Source Development Labs
 * Copyright (c) 2007 Greg Kroah-Hartman <gregkh@suse.de>
 * Copyright (c) 2007 Novell Inc.
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "base.h"

static struct device *next_device(struct klist_iter *i)
{
	struct klist_node *n = klist_next(i);
	struct device *dev = NULL;
	struct device_private *dev_prv;

	if (n) {
		dev_prv = to_device_private_driver(n);
		dev = dev_prv->device;
	}
	return dev;
}

/**
 * driver_for_each_device - Iterator for devices bound to a driver.
 * @drv: Driver we're iterating.
 * @start: Device to begin with
 * @data: Data to pass to the callback.
 * @fn: Function to call for each device.
 *
 * Iterate over the @drv's list of devices calling @fn for each one.
 */
int driver_for_each_device(struct device_driver *drv, struct device *start,
			   void *data, int (*fn)(struct device *, void *))
{
	struct klist_iter i;
	struct device *dev;
	int error = 0;

	if (!drv)
		return -EINVAL;

	klist_iter_init_node(&drv->p->klist_devices, &i,
			     start ? &start->p->knode_driver : NULL);
	while ((dev = next_device(&i)) && !error)
		error = fn(dev, data);
	klist_iter_exit(&i);
	return error;
}
EXPORT_SYMBOL_GPL(driver_for_each_device);

/**
 * driver_find_device - device iterator for locating a particular device.
 * @drv: The device's driver
 * @start: Device to begin with
 * @data: Data to pass to match function
 * @match: Callback function to check device
 *
 * This is similar to the driver_for_each_device() function above, but
 * it returns a reference to a device that is 'found' for later use, as
 * determined by the @match callback.
 *
 * The callback should return 0 if the device doesn't match and non-zero
 * if it does.  If the callback returns non-zero, this function will
 * return to the caller and not iterate over any more devices.
 */
struct device *driver_find_device(struct device_driver *drv,
				  struct device *start, void *data,
				  int (*match)(struct device *dev, void *data))
{
	struct klist_iter i;
	struct device *dev;

	if (!drv)
		return NULL;

	klist_iter_init_node(&drv->p->klist_devices, &i,
			     (start ? &start->p->knode_driver : NULL));
	while ((dev = next_device(&i)))
		if (match(dev, data) && get_device(dev))
			break;
	klist_iter_exit(&i);
	return dev;
}
EXPORT_SYMBOL_GPL(driver_find_device);

/**
 * 在驱动所属目录创建一个属性文件[1]P198.
 *
 * driver_create_file - create sysfs file for driver.
 * @drv: driver.
 * @attr: driver attribute descriptor.
 * 
 * @see Linux驱动开发入门与实战(2)
 */
int driver_create_file(struct device_driver *drv,
		       const struct driver_attribute *attr)
{
	int error;
	if (drv)
		error = sysfs_create_file(&drv->p->kobj, &attr->attr);
	else
		error = -EINVAL;
	return error;
}
EXPORT_SYMBOL_GPL(driver_create_file);

/**
 * 在驱动所属目录删除一个属性文件[1]P198.
 *
 * driver_remove_file - remove sysfs file for driver.
 * @drv: driver.
 * @attr: driver attribute descriptor.
 *
 * @see Linux驱动开发入门与实战(2)
 */
void driver_remove_file(struct device_driver *drv,
			const struct driver_attribute *attr)
{
	if (drv)
		sysfs_remove_file(&drv->p->kobj, &attr->attr);
}
EXPORT_SYMBOL_GPL(driver_remove_file);

/**
 * driver_add_kobj - add a kobject below the specified driver
 * @drv: requesting device driver
 * @kobj: kobject to add below this driver
 * @fmt: format string that names the kobject
 *
 * You really don't want to do this, this is only here due to one looney
 * iseries driver, go poke those developers if you are annoyed about
 * this...
 */
int driver_add_kobj(struct device_driver *drv, struct kobject *kobj,
		    const char *fmt, ...)
{
	va_list args;
	char *name;
	int ret;

	va_start(args, fmt);
	name = kvasprintf(GFP_KERNEL, fmt, args);
	va_end(args);

	if (!name)
		return -ENOMEM;

	ret = kobject_add(kobj, &drv->p->kobj, "%s", name);
	kfree(name);
	return ret;
}
EXPORT_SYMBOL_GPL(driver_add_kobj);

/**
 * get_driver - increment driver reference count.
 * @drv: driver.
 */
struct device_driver *get_driver(struct device_driver *drv)
{
	if (drv) {
		struct driver_private *priv;
		struct kobject *kobj;

		kobj = kobject_get(&drv->p->kobj);
		priv = to_driver(kobj);
		return priv->driver;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(get_driver);

/**
 * put_driver - decrement driver's refcount.
 * @drv: driver.
 */
void put_driver(struct device_driver *drv)
{
	kobject_put(&drv->p->kobj);
}
EXPORT_SYMBOL_GPL(put_driver);

static int driver_add_groups(struct device_driver *drv,
			     const struct attribute_group **groups)
{
	int error = 0;
	int i;

	if (groups) {
		for (i = 0; groups[i]; i++) {
			error = sysfs_create_group(&drv->p->kobj, groups[i]);
			if (error) {
				while (--i >= 0)
					sysfs_remove_group(&drv->p->kobj,
							   groups[i]);
				break;
			}
		}
	}
	return error;
}

static void driver_remove_groups(struct device_driver *drv,
				 const struct attribute_group **groups)
{
	int i;

	if (groups)
		for (i = 0; groups[i]; i++)
			sysfs_remove_group(&drv->p->kobj, groups[i]);
}

/**
 * 设备注册函数[1]P197.
 * 
 * 该函数的功能是向设备驱动模型中插入一个新的device_driver对象,
 * 当注册成功后, 会在sysfs文件系统中创建一个新的目录[1]P197.
 * 
 * driver_register - register driver with bus
 * @drv: driver to register
 *
 * We pass off most of the work to the bus_add_driver() call,
 * since most of the things we have to do deal with the bus
 * structures.
 *
 * @see Linux驱动开发入门与实战(2)
 */
int driver_register(struct device_driver *drv)
{
	int ret; 						/* 返回值[1]P197. */
	struct device_driver *other;

	BUG_ON(!drv->bus->p);

	if ((drv->bus->probe && drv->probe) || 		/* drv和drv所属的bus中只要有一个提供该函数即可， */
	    (drv->bus->remove && drv->remove) ||    /* 否则也只能调用bus的函数， 而不理会drv的函数. */
	    (drv->bus->shutdown && drv->shutdown))  /* 这种方式已经过时, 推荐使用bus_type中的方法[1]P197. */
		printk(KERN_WARNING "Driver '%s' needs updating - please use "
			"bus_type methods\n", drv->name);

	other = driver_find(drv->name, drv->bus); 	/* 总线中是否已经存在该驱动[1]P197. */
	if (other) { 								/* 驱动已经注册, 则返回存在[1]P197. */
		put_driver(other); 						/* 减少驱动引用计数[1]P197. */
		printk(KERN_ERR "Error: Driver '%s' is already registered, "
			"aborting...\n", drv->name);
		return -EBUSY;
	}

	ret = bus_add_driver(drv); 					/* 将本drv注册登记到drv->bus所在的总线[1]P197. */
	if (ret) 									/* 失败则返回[1]P197. */
		return ret;
	ret = driver_add_groups(drv, drv->groups); 	/* 将该驱动添加到所属组中[1]P197. */
	if (ret)
		bus_remove_driver(drv); 				/* 如果错误, 从总线中移除驱动程序[1]P197. */
	return ret;
}
EXPORT_SYMBOL_GPL(driver_register);

/**
 * 该函数用于注销驱动程序[1]P197.
 *
 * driver_unregister - remove driver from system.
 * @drv: driver.
 *
 * Again, we pass off most of the work to the bus-level call.
 * 
 * @see Linux驱动开发入门与实战(2)
 */
void driver_unregister(struct device_driver *drv)
{
	if (!drv || !drv->p) {
		WARN(1, "Unexpected driver unregister!\n");
		return;
	}
	driver_remove_groups(drv, drv->groups);
	bus_remove_driver(drv);
}
EXPORT_SYMBOL_GPL(driver_unregister);

/**
 * 在总线bus中查找驱动name[1]P197.
 *
 * driver_find - locate driver on a bus by its name.
 * @name: name of the driver.
 * @bus: bus to scan for the driver.
 *
 * Call kset_find_obj() to iterate over list of drivers on
 * a bus to find driver by name. Return driver if found.
 *
 * Note that kset_find_obj increments driver's reference count.
 *
 * @see Linux驱动开发入门与实战(2)
 */
struct device_driver *driver_find(const char *name, struct bus_type *bus)
{
	struct kobject *k = kset_find_obj(bus->p->drivers_kset, name);
	struct driver_private *priv;

	if (k) {
		priv = to_driver(k);
		return priv->driver;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(driver_find);
