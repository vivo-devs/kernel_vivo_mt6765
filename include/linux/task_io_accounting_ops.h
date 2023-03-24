/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Task I/O accounting operations
 */
#ifndef __TASK_IO_ACCOUNTING_OPS_INCLUDED
#define __TASK_IO_ACCOUNTING_OPS_INCLUDED

#include <linux/sched.h>

#define IO_MB	(1 * 1024 * 1024)
#define IO_PRINT_UNIT	(100 * IO_MB)
#ifdef CONFIG_TASK_IO_ACCOUNTING
static inline void task_io_print(u64 bytes, u64 *last_bytes, bool rw)
{
	if (bytes && ((bytes - *last_bytes) > IO_PRINT_UNIT)) {
		*last_bytes = bytes;
		pr_err("[%s:%d] <%s:%d> <%s:%d> ppid %d %s %llu bytes(%llu MB)\n",
				__func__, __LINE__,
				current->comm, task_pid_nr(current),
				current->group_leader ? current->group_leader->comm : "",
				task_tgid_nr(current),
				task_pid_nr(rcu_dereference(current->real_parent)),
				rw ? "write" : "read",
				bytes,
				bytes / IO_MB
			);
	}
}

static inline void task_io_account_read(size_t bytes)
{
	current->ioac.read_bytes += bytes;
	if (current->group_leader && (task_tgid_nr(current) != task_pid_nr(current))) {
		current->group_leader->ioac.tg_read_bytes += bytes;
		task_io_print(current->group_leader->ioac.read_bytes
				+ current->group_leader->ioac.tg_read_bytes,
				&current->group_leader->ioac.last_read_bytes, false);
	} else
		task_io_print(current->ioac.read_bytes
				+ current->ioac.tg_read_bytes,
				&current->ioac.last_read_bytes, false);
}

/*
 * We approximate number of blocks, because we account bytes only.
 * A 'block' is 512 bytes
 */
static inline unsigned long task_io_get_inblock(const struct task_struct *p)
{
	return p->ioac.read_bytes >> 9;
}

static inline void task_io_account_write(size_t bytes)
{
	current->ioac.write_bytes += bytes;
	if (current->group_leader && (task_tgid_nr(current) != task_pid_nr(current))) {
		current->group_leader->ioac.tg_write_bytes += bytes;
		task_io_print(current->group_leader->ioac.write_bytes
				+ current->group_leader->ioac.tg_write_bytes,
				&current->group_leader->ioac.last_write_bytes, true);
	} else
		task_io_print(current->ioac.write_bytes
				+ current->ioac.tg_write_bytes,
				&current->ioac.last_write_bytes, true);
}

/*
 * We approximate number of blocks, because we account bytes only.
 * A 'block' is 512 bytes
 */
static inline unsigned long task_io_get_oublock(const struct task_struct *p)
{
	return p->ioac.write_bytes >> 9;
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
	current->ioac.cancelled_write_bytes += bytes;
}

static inline void task_io_accounting_init(struct task_io_accounting *ioac)
{
	memset(ioac, 0, sizeof(*ioac));
}

static inline void task_blk_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	dst->read_bytes += src->read_bytes;
	dst->write_bytes += src->write_bytes;
	dst->cancelled_write_bytes += src->cancelled_write_bytes;
}

#else

static inline void task_io_account_read(size_t bytes)
{
}

static inline unsigned long task_io_get_inblock(const struct task_struct *p)
{
	return 0;
}

static inline void task_io_account_write(size_t bytes)
{
}

static inline unsigned long task_io_get_oublock(const struct task_struct *p)
{
	return 0;
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
}

static inline void task_io_accounting_init(struct task_io_accounting *ioac)
{
}

static inline void task_blk_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
}

#endif /* CONFIG_TASK_IO_ACCOUNTING */

#ifdef CONFIG_TASK_XACCT
static inline void task_chr_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	dst->rchar += src->rchar;
	dst->wchar += src->wchar;
	dst->syscr += src->syscr;
	dst->syscw += src->syscw;
	dst->syscfs += src->syscfs;
}
#else
static inline void task_chr_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
}
#endif /* CONFIG_TASK_XACCT */

static inline void task_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	task_chr_io_accounting_add(dst, src);
	task_blk_io_accounting_add(dst, src);
}
#endif /* __TASK_IO_ACCOUNTING_OPS_INCLUDED */
