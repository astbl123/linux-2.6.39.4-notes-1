
/**
 * bus_type的私有数据[1]P190.
 *
 * struct subsys_private - structure to hold the private to the driver core portions of the bus_type/class structure.
 *
 * @subsys - the struct kset that defines this subsystem
 * @devices_kset - the list of devices associated
 *
 * @drivers_kset - the list of drivers associated
 * @klist_devices - the klist to iterate over the @devices_kset
 * @klist_drivers - the klist to iterate over the @drivers_kset
 * @bus_notifier - the bus notifier list for anything that cares about things
 *                 on this bus.
 * @bus - pointer back to the struct bus_type that this structure is associated
 *        with.
 *
 * @class_interfaces - list of class_interfaces associated
 * @glue_dirs - "glue" directory to put in-between the parent device to
 *              avoid namespace conflicts
 * @class_mutex - mutex to protect the children, devices, and interfaces lists.
 * @class - pointer back to the struct class that this structure is associated
 *          with.
 *
 * This structure is the one that is the actual kobject allowing struct
 * bus_type/class to be statically allocated safely.  Nothing outside of the
 * driver core should ever touch these fields.
 *
 * @see Linux驱动开发入门于实战(2)
 */
struct subsys_private {
	struct kset subsys;    			/*!< 代表bus子系统, 里面的kobj是该bus的主kobj, 也就是最顶层[1]P190. */
	struct kset *devices_kset; 		/*!< 挂接到该总线上的所有设备集合[1]P190. */

	struct kset *drivers_kset; 		/*!< 挂接到该总线上的所有驱动集合集合[1]P190. */
	struct klist klist_devices; 	/*!< 所有设备列表, 与devices_kset中的list相同[1]P190. */
	struct klist klist_drivers; 	/*!< 所有驱动列表, 与drivers_kset中的list相同[1]P190. */
	struct blocking_notifier_head bus_notifier;
	unsigned int drivers_autoprobe:1; /*!< 设置是否在驱动注册时, 自动探测(probe)设备[1]P190. */
	struct bus_type *bus; 			/*!< 回指包含自己的总线[1]P190. */

	struct list_head class_interfaces;
	struct kset glue_dirs;
	struct mutex class_mutex;
	struct class *class;
};
#define to_subsys_private(obj) container_of(obj, struct subsys_private, subsys.kobj)

/**
 * 驱动私有数据[1]P196.
 *
 * @see Linux驱动开发入门于实战(2)
 */
struct driver_private {
	struct kobject kobj; 			/*!< 内嵌的kobject结构, 用来构建设备驱动模型的结构[1]P196. */
	struct klist klist_devices; 	/*!< 该驱动支持的所有设备链表[1]P196. */
	struct klist_node knode_bus; 	/*!< 该驱动所属总线[1]P196. */
	struct module_kobject *mkobj; 	/*!< 驱动的模块[1]P196. */
	struct device_driver *driver; 	/*!< 指向驱动本身[1]P196. */
};
#define to_driver(obj) container_of(obj, struct driver_private, kobj)

/**
 * struct device_private - structure to hold the private to the driver core portions of the device structure.
 *
 * @klist_children - klist containing all children of this device
 * @knode_parent - node in sibling list
 * @knode_driver - node in driver list
 * @knode_bus - node in bus list
 * @driver_data - private pointer for driver specific info.  Will turn into a
 * list soon.
 * @device - pointer back to the struct class that this structure is
 * associated with.
 *
 * Nothing outside of the driver core should ever touch these fields.
 */
struct device_private {
	struct klist klist_children;
	struct klist_node knode_parent;
	struct klist_node knode_driver;
	struct klist_node knode_bus;
	void *driver_data;
	struct device *device;
};
#define to_device_private_parent(obj)	\
	container_of(obj, struct device_private, knode_parent)
#define to_device_private_driver(obj)	\
	container_of(obj, struct device_private, knode_driver)
#define to_device_private_bus(obj)	\
	container_of(obj, struct device_private, knode_bus)

extern int device_private_init(struct device *dev);

/* initialisation functions */
extern int devices_init(void);
extern int buses_init(void);
extern int classes_init(void);
extern int firmware_init(void);
#ifdef CONFIG_SYS_HYPERVISOR
extern int hypervisor_init(void);
#else
static inline int hypervisor_init(void) { return 0; }
#endif
extern int platform_bus_init(void);
extern int system_bus_init(void);
extern int cpu_dev_init(void);

extern int bus_add_device(struct device *dev);
extern void bus_probe_device(struct device *dev);
extern void bus_remove_device(struct device *dev);

extern int bus_add_driver(struct device_driver *drv);
extern void bus_remove_driver(struct device_driver *drv);

extern void driver_detach(struct device_driver *drv);
extern int driver_probe_device(struct device_driver *drv, struct device *dev);
static inline int driver_match_device(struct device_driver *drv,
				      struct device *dev)
{
	return drv->bus->match ? drv->bus->match(dev, drv) : 1;
}

extern void sysdev_shutdown(void);

extern char *make_class_name(const char *name, struct kobject *kobj);

extern int devres_release_all(struct device *dev);

extern struct kset *devices_kset;

#if defined(CONFIG_MODULES) && defined(CONFIG_SYSFS)
extern void module_add_driver(struct module *mod, struct device_driver *drv);
extern void module_remove_driver(struct device_driver *drv);
#else
static inline void module_add_driver(struct module *mod,
				     struct device_driver *drv) { }
static inline void module_remove_driver(struct device_driver *drv) { }
#endif

#ifdef CONFIG_DEVTMPFS
extern int devtmpfs_init(void);
#else
static inline int devtmpfs_init(void) { return 0; }
#endif
