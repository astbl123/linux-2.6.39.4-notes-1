#ifndef _ASMARM_CURRENT_H
#define _ASMARM_CURRENT_H

#include <linux/thread_info.h>

static inline struct task_struct *get_current(void) __attribute_const__;

static inline struct task_struct *get_current(void)
{
	return current_thread_info()->task;
}

/**
 * 指向当前正在运行的进程[1]P27.
 *
 * @see LINUX设备驱动程序(第三版)
 */
#define current (get_current())

#endif /* _ASMARM_CURRENT_H */
