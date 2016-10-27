/*
 * kobject.h - generic kernel object infrastructure.
 *
 * Copyright (c) 2002-2003 Patrick Mochel
 * Copyright (c) 2002-2003 Open Source Development Labs
 * Copyright (c) 2006-2008 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (c) 2006-2008 Novell Inc.
 *
 * This file is released under the GPLv2.
 *
 * Please read Documentation/kobject.txt before using the kobject
 * interface, ESPECIALLY the parts about reference counts and object
 * destructors.
 */

#ifndef _KOBJECT_H_
#define _KOBJECT_H_

#include <linux/types.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/compiler.h>
#include <linux/spinlock.h>
#include <linux/kref.h>
#include <linux/kobject_ns.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <asm/atomic.h>

#define UEVENT_HELPER_PATH_LEN		256
#define UEVENT_NUM_ENVP			32	/* number of env pointers */
#define UEVENT_BUFFER_SIZE		2048	/* buffer for the variables */

/* path to the userspace helper executed on an event */
extern char uevent_helper[];

/* counter to tag the uevent, read only except for the kobject core */
extern u64 uevent_seqnum;

/*
 * The actions here must match the index to the string array
 * in lib/kobject_uevent.c
 *
 * Do not add new actions here without checking with the driver-core
 * maintainers. Action strings are not meant to express subsystem
 * or device specific properties. In most cases you want to send a
 * kobject_uevent_env(kobj, KOBJ_CHANGE, env) with additional event
 * specific variables added to the event environment.
 */
enum kobject_action {
	KOBJ_ADD,
	KOBJ_REMOVE,
	KOBJ_CHANGE,
	KOBJ_MOVE,
	KOBJ_ONLINE,
	KOBJ_OFFLINE,
	KOBJ_MAX
};

/**
 * kobject始终代表sysfs文件系统中的一个目录,
 * 而不是文件[1]P176. 
 * 
 * kobject一般包含于一个更大的自定义结构中[1]P178.
 *
 * kobject可以看作是所有总线, 设备和驱动的抽象基类,
 * 一个kobject对应sysfs中的1个目录[2]P125.
 * 
 * @see [1]Linux驱动开发入门与实战(2)
 * @see [2]Linux设备驱动开发详解(3)
 */
struct kobject {
	const char		*name;  			/*!< 是sysfs文件系统中的目录名, 通常使用kobject_set_name()设置[1]P176. */
	struct list_head	entry; 			/*!< 连接下一个kobject结构[1]P172. */
	struct kobject		*parent; 		/*!< 父kobject[1]P172, 指明了该kobject在sysfs文件系统中的位置[1]P176. */
	struct kset		*kset;
	struct kobj_type	*ktype;   		/*!< kobject的属性, 用文件来表示, 放在kobject对应的目录下[1]P176. */
	struct sysfs_dirent	*sd; 			/*!< 对应sysfs的文件目录[1]P172. */
	struct kref		kref; 				/*!< kobject的引用计数, 为0时内核将会调用release()函数释放该kobject[1]P173. */
	unsigned int state_initialized:1; 	/*!< 表示kobject是否已经初始化过, ":1"表示只使用unsigned int的最低1位表示这个布尔值[1]P173. */
	unsigned int state_in_sysfs:1; 		/*!< 是否已经加入sysfs中[1]P172. */
	unsigned int state_add_uevent_sent:1;
	unsigned int state_remove_uevent_sent:1;
	unsigned int uevent_suppress:1;
};

extern int kobject_set_name(struct kobject *kobj, const char *name, ...)
			    __attribute__((format(printf, 2, 3)));
extern int kobject_set_name_vargs(struct kobject *kobj, const char *fmt,
				  va_list vargs);

static inline const char *kobject_name(const struct kobject *kobj)
{
	return kobj->name;
}

extern void kobject_init(struct kobject *kobj, struct kobj_type *ktype);
extern int __must_check kobject_add(struct kobject *kobj,
				    struct kobject *parent,
				    const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
extern int __must_check kobject_init_and_add(struct kobject *kobj,
					     struct kobj_type *ktype,
					     struct kobject *parent,
					     const char *fmt, ...)
	__attribute__((format(printf, 4, 5)));

extern void kobject_del(struct kobject *kobj);

extern struct kobject * __must_check kobject_create(void);
extern struct kobject * __must_check kobject_create_and_add(const char *name,
						struct kobject *parent);

extern int __must_check kobject_rename(struct kobject *, const char *new_name);
extern int __must_check kobject_move(struct kobject *, struct kobject *);

extern struct kobject *kobject_get(struct kobject *kobj);
extern void kobject_put(struct kobject *kobj);

extern char *kobject_get_path(struct kobject *kobj, gfp_t flag);


/**
 * 每个kobject对象都有一些属性,这些属性由kobject结构体表示.
 * 最开始,内核开发者考虑将属性包含在kobject结构体中,
 * 后来考虑到同类设备会具有相同的属性,所以将属性隔离开来,
 * 由kobj_type表示[1]P175.
 *
 * @see [1]Linux驱动开发入门与实战(2)
 */
struct kobj_type {
	void (*release)(struct kobject *kobj);  /*!< kobject引用计数为0时, 由内核自动调用, 用于释放kobject和其占有的资源[1]P176,178. */
	const struct sysfs_ops *sysfs_ops; 		/*!< 对属性的操作函数, 只有读操作和写操作两种[1]P176. */
	struct attribute **default_attrs; 		/*! 属性数组[1]P176. */
	const struct kobj_ns_type_operations *(*child_ns_type)(struct kobject *kobj);
	const void *(*namespace)(struct kobject *kobj);
};

struct kobj_uevent_env {
	char *envp[UEVENT_NUM_ENVP];
	int envp_idx;
	char buf[UEVENT_BUFFER_SIZE];
	int buflen;
};

/*!
 * 热插拔事件是从内核空间发送到用户空间的通知, 表明系统某些部分的配置已经发生变化.
 * 用户空间接收到内核空间的通知后, 会调用相应的程序, 处理配置的变化. 例如当U盘插入
 * 到USB系统时, 会产生一个热插拔事件, 内核会捕获这个热插拔事件, 并调用用户空间的
 * /sbin/hotplug程序, 该程序通过加载驱动程序来相应U盘插入的动作[1]P180.
 *
 * 内核将在什么时候产生热插拔事件呢? 当驱动程序将kobject注册到设备驱动模型时, 会产生 
 * 这些事件. 也就是当内核调用kobject_add()和kobject_del()函数时, 会产生热插拔事件[1]P180.
 * 
 * 热插拔事件产生时, 内核会根据kobject的kset指针找到所属的kset结构体, 执行kset结构体中
 * 的uevent_ops包含的热插拔函数[1]P180.
 *
 * @see Linux驱动开发入门与实战(2)
 */
struct kset_uevent_ops {
	int (* const filter)(struct kset *kset, struct kobject *kobj); 		 /*!< 在把事件发送给用户空间时, 过滤不需要产生的事件[1]P180. */
	const char *(* const name)(struct kset *kset, struct kobject *kobj); /*!< 向用户空间返回子系统名字[1]P181. */
	int (* const uevent)(struct kset *kset, struct kobject *kobj, 		
		      struct kobj_uevent_env *env); 							 /*!< 在热插拔程序执行前, 向环境变量中写入值[1]P181. */
};

struct kobj_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count);
};

extern const struct sysfs_ops kobj_sysfs_ops;

struct sock;

/**
 * kobject通过kset组织成层次化的结构, kset是具有相同类型kobject的集合,
 * 像驱动程序一样放在/sys/drivers/目录下, 目录drivers是一个kset对象,
 * 包含系统中的驱动程序对应的目录, 驱动程序的目录由kobject表示[1]P179.
 *
 * kset于kobject的关系详见参考资料[1]P181.
 * 
 * struct kset - a set of kobjects of a specific type, belonging to a specific subsystem.
 *
 * A kset defines a group of kobjects.  They can be individually
 * different "types" but overall these kobjects all want to be grouped
 * together and operated on in the same manner.  ksets are used to
 * define the attribute callbacks and other common events that happen to
 * a kobject.
 *
 * @list: the list of all kobjects for this kset
 * @list_lock: a lock for iterating over the kobjects
 * @kobj: the embedded kobject for this kset (recursion, isn't it fun...)
 * @uevent_ops: the set of uevent operations for this kset.  These are
 * called whenever a kobject has something happen to it so that the kset
 * can add new environment variables, or filter out the uevents if so
 * desired.
 *
 * @see [1]Linux驱动开发入门与实战(2)
 */
struct kset {
	struct list_head list; /*!< 连接所包含的kobject对象的链表首部[1]P180. */
	spinlock_t list_lock;  /*!< 保护list链表的自旋锁[1]P180. */
	struct kobject kobj;   /*!< 内嵌的kobject, 说明kset本身也是个目录, kset的引用计数也就是kobject的引用计数[1]P180. */
	const struct kset_uevent_ops *uevent_ops; /*!< 热插拔事件函数集[1]P180. */
};

extern void kset_init(struct kset *kset);
extern int __must_check kset_register(struct kset *kset);
extern void kset_unregister(struct kset *kset);
extern struct kset * __must_check kset_create_and_add(const char *name,
						const struct kset_uevent_ops *u,
						struct kobject *parent_kobj);

static inline struct kset *to_kset(struct kobject *kobj)
{
	return kobj ? container_of(kobj, struct kset, kobj) : NULL;
}

static inline struct kset *kset_get(struct kset *k)
{
	return k ? to_kset(kobject_get(&k->kobj)) : NULL;
}

static inline void kset_put(struct kset *k)
{
	kobject_put(&k->kobj);
}

static inline struct kobj_type *get_ktype(struct kobject *kobj)
{
	return kobj->ktype;
}

extern struct kobject *kset_find_obj(struct kset *, const char *);
extern struct kobject *kset_find_obj_hinted(struct kset *, const char *,
						struct kobject *);

/* The global /sys/kernel/ kobject for people to chain off of */
extern struct kobject *kernel_kobj;
/* The global /sys/kernel/mm/ kobject for people to chain off of */
extern struct kobject *mm_kobj;
/* The global /sys/hypervisor/ kobject for people to chain off of */
extern struct kobject *hypervisor_kobj;
/* The global /sys/power/ kobject for people to chain off of */
extern struct kobject *power_kobj;
/* The global /sys/firmware/ kobject for people to chain off of */
extern struct kobject *firmware_kobj;

#if defined(CONFIG_HOTPLUG)
int kobject_uevent(struct kobject *kobj, enum kobject_action action);
int kobject_uevent_env(struct kobject *kobj, enum kobject_action action,
			char *envp[]);

int add_uevent_var(struct kobj_uevent_env *env, const char *format, ...)
	__attribute__((format (printf, 2, 3)));

int kobject_action_type(const char *buf, size_t count,
			enum kobject_action *type);
#else
static inline int kobject_uevent(struct kobject *kobj,
				 enum kobject_action action)
{ return 0; }
static inline int kobject_uevent_env(struct kobject *kobj,
				      enum kobject_action action,
				      char *envp[])
{ return 0; }

static inline __attribute__((format(printf, 2, 3)))
int add_uevent_var(struct kobj_uevent_env *env, const char *format, ...)
{ return 0; }

static inline int kobject_action_type(const char *buf, size_t count,
				      enum kobject_action *type)
{ return -EINVAL; }
#endif

#endif /* _KOBJECT_H_ */
