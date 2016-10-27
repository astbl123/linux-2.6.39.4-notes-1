#ifndef _LINUX_CDEV_H
#define _LINUX_CDEV_H

#include <linux/kobject.h>
#include <linux/kdev_t.h>
#include <linux/list.h>

struct file_operations;
struct inode;
struct module;

/**
 * Linux内核中使用cdev描述一个字符设备[1]P134.
 *
 * @see Linux设备驱动开发详解(3)
 */
struct cdev {
	struct kobject kobj; 	/*!< 内嵌的kobject模块[1]P134. */
	struct module *owner; 	/*!< 所属模块[1]P134. */
	const struct file_operations *ops; /*!< 文件操作结构体[1]P134. */
	struct list_head list; 	/*!< */
	dev_t dev; 				/*!< 设备号[1]P134. */
	unsigned int count;
};

void cdev_init(struct cdev *, const struct file_operations *);

struct cdev *cdev_alloc(void);

void cdev_put(struct cdev *p);

int cdev_add(struct cdev *, dev_t, unsigned);

void cdev_del(struct cdev *);

void cd_forget(struct inode *);

extern struct backing_dev_info directly_mappable_cdev_bdi;

#endif
