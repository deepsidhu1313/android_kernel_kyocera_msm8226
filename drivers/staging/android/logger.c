/*
 * drivers/misc/logger.c
 *
 * A Logging Subsystem
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C)2015 KYOCERA Corporation
 */

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <mach/msm_smem.h>
#include "resetlog.h"
#include "logger.h"

#include <asm/ioctls.h>

#ifndef CONFIG_LOGCAT_SIZE
#define CONFIG_LOGCAT_SIZE 256
#endif

extern void rtc_time_to_tm(unsigned long time, struct rtc_time *tm);

/*
 * struct logger_log - represents a specific log, such as 'main' or 'radio'
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * mutex 'mutex'.
 */
struct logger_log {
	unsigned char		*buffer;/* the ring buffer itself */
	struct miscdevice	misc;	/* misc device representing the log */
	wait_queue_head_t	wq;	/* wait queue for readers */
	struct list_head	readers; /* this log's readers */
	struct mutex		mutex;	/* mutex protecting buffer */
	size_t			w_off;	/* current write head offset */
	size_t			head;	/* new readers start here */
	size_t			size;	/* size of the log */
	struct logger_log_info  *log_info;
};

#define DEFINE_LOGGER_DEVICE(VAR, ADDR, NAME, SIZE, LOGGER_INFO, LOGGER_FOPS) \
static struct logger_log VAR = { \
        .buffer = ADDR, \
        .misc = { \
                .minor = MISC_DYNAMIC_MINOR, \
                .name = NAME, \
                .fops = LOGGER_FOPS, \
                .parent = NULL, \
        }, \
        .wq = __WAIT_QUEUE_HEAD_INITIALIZER(VAR .wq), \
        .readers = LIST_HEAD_INIT(VAR .readers), \
        .mutex = __MUTEX_INITIALIZER(VAR .mutex), \
        .w_off = 0, \
        .head = 0, \
        .size = SIZE, \
        .log_info = LOGGER_INFO, \
}

#if 0
#define EMMC_INDEX_MAX               (5)
#define EMMC_SECTOR_BLK_SIZE         (512)   // eMMC Sector Size

#define SIZE_TO_SECTOR(size)         (((size % EMMC_SECTOR_BLK_SIZE) > 0) ? \
                                      ( size / EMMC_SECTOR_BLK_SIZE) + 1  : \
                                      ( size / EMMC_SECTOR_BLK_SIZE))

#define EMMC_INFO_SECTOR_SIZE        (SIZE_TO_SECTOR(sizeof(emmc_info_type)))
#define EMMC_INFO_START_SECTOR       (0)

#define EMMC_REC_SIZE                (0x1C0000)
#define EMMC_REC_SECTOR_SIZE         (SIZE_TO_SECTOR(EMMC_REC_SIZE))

#define EMMC_REC_START_SECTOR(idx)   ((EMMC_INFO_START_SECTOR + EMMC_INFO_SECTOR_SIZE) + \
                                      (EMMC_REC_SECTOR_SIZE * idx))

#define EMMC_BLOCK_DEV_NAME_MIBIB       ("/dev/block/mmcblk0")
#define EMMC_BLOCK_DEV_NAME             ("/dev/block/mmcblk0p")

#define EMMC_TRUE                           (10)
#define EMMC_FALSE                          (0)
#define EMMC_INITIALIZED                    (10)

#define EMMC_DRV_BOOT_REC_SIG               (0xAA55)
#define EMMC_DRV_MBR_ENTRIES                (4)

#define EMMC_DRV_ERR_PARTITION_NUM          (0xFF)
#define EMMC_DEV_ERROR						(-1)
#define EMMC_NO_ERROR					    (0)

typedef enum
{
    EMMC_DEV_READ    = 0,
    EMMC_DEV_WRITE,
    EMMC_DEV_MAX
} emmc_dev_rw_type;

typedef struct _uint64_st {
    uint32_t low32;
    uint32_t high32;
} uint64_st;

struct emmc_drv_partition_entry_st
{
    uint8_t status;
    uint8_t rsvd0[3];
    uint8_t type;
    uint8_t rsvd1[3];
    uint32_t start_sector;
    uint32_t partition_size;
} __attribute__((__packed__));

typedef struct emmc_drv_partition_entry_st emmc_drv_partition_entry;

struct emmc_drv_boot_record_st {
    uint8_t                   rsvd0[446];
    emmc_drv_partition_entry entry[4];
    uint16_t                  sig;
} __attribute__((__packed__));

typedef struct emmc_drv_boot_record_st emmc_drv_boot_record;

typedef struct emmc_drv_GPT_header {
    uint8_t signature[8];
    uint8_t revision[4];
    uint32_t size;
    uint32_t crc32;
    uint8_t reserved[4];
    uint64_st current_LBA;
    uint64_st backup_LBA;
    uint64_st first_LBA;
    uint64_st last_LBA;
    uint8_t disk_guid[16];
    uint64_st entry_LBA;
    uint32_t entry_num;
    uint32_t entry_size;
    uint32_t entry_array_crc32;
} emmc_drv_GPT_header;

typedef struct emmc_drv_GPT_partition_entry_st {
    uint8_t type_guid[16];
    uint8_t unique_guid[16];
    uint64_st first_LBA;
    uint64_st last_LBA;
    uint64_st attribute_flags;
    uint16_t name[36];
} emmc_drv_GPT_partition_entry;

typedef struct emmc_drv_LBA_st {
    union {
        uint8_t raw[EMMC_SECTOR_BLK_SIZE];
        emmc_drv_boot_record boot_record;
        emmc_drv_GPT_header gpt_header;
        emmc_drv_GPT_partition_entry gpt_partition_entry[4];
    } as;
} emmc_drv_LBA;


/* ResetLog control info */
typedef struct {
    unsigned char               crash_system[CRASH_CODE_SIZE]; // Crash System
    unsigned char               crash_kind[CRASH_CODE_SIZE];   // Crash Kind
    unsigned char               crash_time[CRASH_TIME_SIZE];   // Crash Time
    unsigned char               linux_ver[VERSION_SIZE];       // Linux version
    unsigned char               modem_ver[VERSION_SIZE];       // AMSS version
    unsigned char               model[MODEL_SIZE];             // model string
    unsigned long               kernel_log_start;              // Kernel log top
    unsigned long               kernel_log_size;               // Kernel log size
    struct logger_log_info      logger[LOGGER_INFO_MAX];
    unsigned long               f3_msg_buff_head;              // F3 message read pointer
    unsigned long               f3_msg_trace_wrap_flg;         // F3 message wrap flag
    unsigned long               qcom_err_data_size;            // QCOM error data size
    unsigned long               pSmem_log_write_idx;           // smem log write index
    unsigned long               Sizeof_Smem_log;               // smem log size
} emmc_rec_info_type;

typedef struct {
    unsigned char          herader_ver[HEADER_VERION_SIZE];    // resetlog.h version
    unsigned long          write_index;                        // Index of write (0 - (EMMC_INDEX_MAX - 1))
    unsigned char          valid_flg[EMMC_INDEX_MAX];          // valid(1) or invalid(0)
    emmc_rec_info_type     rec_info[EMMC_INDEX_MAX];           // Record Info
} emmc_info_type;
emmc_info_type          Emmc_log_info;
#endif

/*
 * struct logger_reader - a logging device open for reading
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->mutex.
 */
struct logger_reader {
	struct logger_log	*log;	/* associated log */
	struct list_head	list;	/* entry in logger_log's list */
	size_t			r_off;	/* current read head offset */
	bool			r_all;	/* reader can read all entries */
	int			r_ver;	/* reader ABI version */
};

/* logger_offset - returns index 'n' into the log via (optimized) modulus */
size_t logger_offset(struct logger_log *log, size_t n)
{
	return n & (log->size-1);
}


/*
 * file_get_log - Given a file structure, return the associated log
 *
 * This isn't aesthetic. We have several goals:
 *
 *	1) Need to quickly obtain the associated log during an I/O operation
 *	2) Readers need to maintain state (logger_reader)
 *	3) Writers need to be very fast (open() should be a near no-op)
 *
 * In the reader case, we can trivially go file->logger_reader->logger_log.
 * For a writer, we don't want to maintain a logger_reader, so we just go
 * file->logger_log. Thus what file->private_data points at depends on whether
 * or not the file was opened for reading. This function hides that dirtiness.
 */
static inline struct logger_log *file_get_log(struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;
		return reader->log;
	} else
		return file->private_data;
}

/*
 * get_entry_header - returns a pointer to the logger_entry header within
 * 'log' starting at offset 'off'. A temporary logger_entry 'scratch' must
 * be provided. Typically the return value will be a pointer within
 * 'logger->buf'.  However, a pointer to 'scratch' may be returned if
 * the log entry spans the end and beginning of the circular buffer.
 */
static struct logger_entry *get_entry_header(struct logger_log *log,
		size_t off, struct logger_entry *scratch)
{
	size_t len = min(sizeof(struct logger_entry), log->size - off);
	if (len != sizeof(struct logger_entry)) {
		memcpy(((void *) scratch), log->buffer + off, len);
		memcpy(((void *) scratch) + len, log->buffer,
			sizeof(struct logger_entry) - len);
		return scratch;
	}

	return (struct logger_entry *) (log->buffer + off);
}

/*
 * get_entry_msg_len - Grabs the length of the message of the entry
 * starting from from 'off'.
 *
 * An entry length is 2 bytes (16 bits) in host endian order.
 * In the log, the length does not include the size of the log entry structure.
 * This function returns the size including the log entry structure.
 *
 * Caller needs to hold log->mutex.
 */
static __u32 get_entry_msg_len(struct logger_log *log, size_t off)
{
	struct logger_entry scratch;
	struct logger_entry *entry;

	entry = get_entry_header(log, off, &scratch);
	return entry->len;
}

static size_t get_user_hdr_len(int ver)
{
	if (ver < 2)
		return sizeof(struct user_logger_entry_compat);
	else
		return sizeof(struct logger_entry);
}

static ssize_t copy_header_to_user(int ver, struct logger_entry *entry,
					 char __user *buf)
{
	void *hdr;
	size_t hdr_len;
	struct user_logger_entry_compat v1;

	if (ver < 2) {
		v1.len      = entry->len;
		v1.__pad    = 0;
		v1.pid      = entry->pid;
		v1.tid      = entry->tid;
		v1.sec      = entry->sec;
		v1.nsec     = entry->nsec;
		hdr         = &v1;
		hdr_len     = sizeof(struct user_logger_entry_compat);
	} else {
		hdr         = entry;
		hdr_len     = sizeof(struct logger_entry);
	}

	return copy_to_user(buf, hdr, hdr_len);
}

/*
 * do_read_log_to_user - reads exactly 'count' bytes from 'log' into the
 * user-space buffer 'buf'. Returns 'count' on success.
 *
 * Caller must hold log->mutex.
 */
static ssize_t do_read_log_to_user(struct logger_log *log,
				   struct logger_reader *reader,
				   char __user *buf,
				   size_t count)
{
	struct logger_entry scratch;
	struct logger_entry *entry;
	size_t len;
	size_t msg_start;

	/*
	 * First, copy the header to userspace, using the version of
	 * the header requested
	 */
	entry = get_entry_header(log, reader->r_off, &scratch);
	if (copy_header_to_user(reader->r_ver, entry, buf))
		return -EFAULT;

	count -= get_user_hdr_len(reader->r_ver);
	buf += get_user_hdr_len(reader->r_ver);
	msg_start = logger_offset(log,
		reader->r_off + sizeof(struct logger_entry));

	/*
	 * We read from the msg in two disjoint operations. First, we read from
	 * the current msg head offset up to 'count' bytes or to the end of
	 * the log, whichever comes first.
	 */
	len = min(count, log->size - msg_start);
	if (copy_to_user(buf, log->buffer + msg_start, len))
		return -EFAULT;

	/*
	 * Second, we read any remaining bytes, starting back at the head of
	 * the log.
	 */
	if (count != len)
		if (copy_to_user(buf + len, log->buffer, count - len))
			return -EFAULT;

	reader->r_off = logger_offset(log, reader->r_off +
		sizeof(struct logger_entry) + count);

	return count + get_user_hdr_len(reader->r_ver);
}

/*
 * get_next_entry_by_uid - Starting at 'off', returns an offset into
 * 'log->buffer' which contains the first entry readable by 'euid'
 */
static size_t get_next_entry_by_uid(struct logger_log *log,
		size_t off, uid_t euid)
{
	while (off != log->w_off) {
		struct logger_entry *entry;
		struct logger_entry scratch;
		size_t next_len;

		entry = get_entry_header(log, off, &scratch);

		if (entry->euid == euid)
			return off;

		next_len = sizeof(struct logger_entry) + entry->len;
		off = logger_offset(log, off + next_len);
	}

	return off;
}

/*
 * logger_read - our log's read() method
 *
 * Behavior:
 *
 *	- O_NONBLOCK works
 *	- If there are no log entries to read, blocks until log is written to
 *	- Atomically reads exactly one log entry
 *
 * Will set errno to EINVAL if read
 * buffer is insufficient to hold next entry.
 */
static ssize_t logger_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
	struct logger_reader *reader = file->private_data;
	struct logger_log *log = reader->log;
	ssize_t ret;
	DEFINE_WAIT(wait);

start:
	while (1) {
		mutex_lock(&log->mutex);

		prepare_to_wait(&log->wq, &wait, TASK_INTERRUPTIBLE);

		ret = (log->w_off == reader->r_off);
		mutex_unlock(&log->mutex);
		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		schedule();
	}

	finish_wait(&log->wq, &wait);
	if (ret)
		return ret;

	mutex_lock(&log->mutex);

	if (!reader->r_all)
		reader->r_off = get_next_entry_by_uid(log,
			reader->r_off, current_euid());

	/* is there still something to read or did we race? */
	if (unlikely(log->w_off == reader->r_off)) {
		mutex_unlock(&log->mutex);
		goto start;
	}

	/* get the size of the next entry */
	ret = get_user_hdr_len(reader->r_ver) +
		get_entry_msg_len(log, reader->r_off);
	if (count < ret) {
		ret = -EINVAL;
		goto out;
	}

	/* get exactly one entry from the log */
	ret = do_read_log_to_user(log, reader, buf, ret);

out:
	mutex_unlock(&log->mutex);

	return ret;
}

/*
 * get_next_entry - return the offset of the first valid entry at least 'len'
 * bytes after 'off'.
 *
 * Caller must hold log->mutex.
 */
static size_t get_next_entry(struct logger_log *log, size_t off, size_t len)
{
	size_t count = 0;

	do {
		size_t nr = sizeof(struct logger_entry) +
			get_entry_msg_len(log, off);
		off = logger_offset(log, off + nr);
		count += nr;
	} while (count < len);

	return off;
}

/*
 * is_between - is a < c < b, accounting for wrapping of a, b, and c
 *    positions in the buffer
 *
 * That is, if a<b, check for c between a and b
 * and if a>b, check for c outside (not between) a and b
 *
 * |------- a xxxxxxxx b --------|
 *               c^
 *
 * |xxxxx b --------- a xxxxxxxxx|
 *    c^
 *  or                    c^
 */
static inline int is_between(size_t a, size_t b, size_t c)
{
	if (a < b) {
		/* is c between a and b? */
		if (a < c && c <= b)
			return 1;
	} else {
		/* is c outside of b through a? */
		if (c <= b || a < c)
			return 1;
	}

	return 0;
}

/*
 * fix_up_readers - walk the list of all readers and "fix up" any who were
 * lapped by the writer; also do the same for the default "start head".
 * We do this by "pulling forward" the readers and start head to the first
 * entry after the new write head.
 *
 * The caller needs to hold log->mutex.
 */
static void fix_up_readers(struct logger_log *log, size_t len)
{
	size_t old = log->w_off;
	size_t new = logger_offset(log, old + len);
	struct logger_reader *reader;

	if (is_between(old, new, log->head))
	{
		log->head = get_next_entry(log, log->head, len);
		log->log_info->head = log->head;
	}

	list_for_each_entry(reader, &log->readers, list)
		if (is_between(old, new, reader->r_off))
			reader->r_off = get_next_entry(log, reader->r_off, len);
}

/*
 * do_write_log - writes 'len' bytes from 'buf' to 'log'
 *
 * The caller needs to hold log->mutex.
 */
static void do_write_log(struct logger_log *log, const void *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	memcpy(log->buffer + log->w_off, buf, len);

	if (count != len)
		memcpy(log->buffer, buf + len, count - len);

	log->w_off = logger_offset(log, log->w_off + count);
	log->log_info->w_off = log->w_off;
}

/*
 * do_write_log_user - writes 'len' bytes from the user-space buffer 'buf' to
 * the log 'log'
 *
 * The caller needs to hold log->mutex.
 *
 * Returns 'count' on success, negative error code on failure.
 */
static ssize_t do_write_log_from_user(struct logger_log *log,
				      const void __user *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	if (len && copy_from_user(log->buffer + log->w_off, buf, len))
		return -EFAULT;

	if (count != len)
		if (copy_from_user(log->buffer, buf + len, count - len))
			/*
			 * Note that by not updating w_off, this abandons the
			 * portion of the new entry that *was* successfully
			 * copied, just above.  This is intentional to avoid
			 * message corruption from missing fragments.
			 */
			return -EFAULT;

	log->w_off = logger_offset(log, log->w_off + count);
	log->log_info->w_off = log->w_off;

	return count;
}

/*
 * logger_aio_write - our write method, implementing support for write(),
 * writev(), and aio_write(). Writes are our fast path, and we try to optimize
 * them above all else.
 */
ssize_t logger_aio_write(struct kiocb *iocb, const struct iovec *iov,
			 unsigned long nr_segs, loff_t ppos)
{
	struct logger_log *log = file_get_log(iocb->ki_filp);
	size_t orig = log->w_off;
	struct logger_entry header;
	struct timespec now;
	ssize_t ret = 0;

	now = current_kernel_time();

	header.pid = current->tgid;
	header.tid = current->pid;
	header.sec = now.tv_sec;
	header.nsec = now.tv_nsec;
	header.euid = current_euid();
	header.len = min_t(size_t, iocb->ki_left, LOGGER_ENTRY_MAX_PAYLOAD);
	header.hdr_size = sizeof(struct logger_entry);

	/* null writes succeed, return zero */
	if (unlikely(!header.len))
		return 0;

	mutex_lock(&log->mutex);

	/*
	 * Fix up any readers, pulling them forward to the first readable
	 * entry after (what will be) the new write offset. We do this now
	 * because if we partially fail, we can end up with clobbered log
	 * entries that encroach on readable buffer.
	 */
	fix_up_readers(log, sizeof(struct logger_entry) + header.len);

	do_write_log(log, &header, sizeof(struct logger_entry));

	while (nr_segs-- > 0) {
		size_t len;
		ssize_t nr;

		/* figure out how much of this vector we can keep */
		len = min_t(size_t, iov->iov_len, header.len - ret);

		/* write out this segment's payload */
		nr = do_write_log_from_user(log, iov->iov_base, len);
		if (unlikely(nr < 0)) {
			log->w_off = orig;
			log->log_info->w_off = log->w_off;
			mutex_unlock(&log->mutex);
			return nr;
		}

		iov++;
		ret += nr;
	}

	mutex_unlock(&log->mutex);

	/* wake up any blocked readers */
	wake_up_interruptible(&log->wq);

	return ret;
}

static struct logger_log *get_log_from_minor(int);

/*
 * logger_open - the log's open() file operation
 *
 * Note how near a no-op this is in the write-only case. Keep it that way!
 */
static int logger_open(struct inode *inode, struct file *file)
{
	struct logger_log *log;
	int ret;

	ret = nonseekable_open(inode, file);
	if (ret)
		return ret;

	log = get_log_from_minor(MINOR(inode->i_rdev));
	if (!log)
		return -ENODEV;

	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader;

		reader = kmalloc(sizeof(struct logger_reader), GFP_KERNEL);
		if (!reader)
			return -ENOMEM;

		reader->log = log;
		reader->r_ver = 1;
		reader->r_all = in_egroup_p(inode->i_gid) ||
			capable(CAP_SYSLOG);

		INIT_LIST_HEAD(&reader->list);

		mutex_lock(&log->mutex);
		reader->r_off = log->head;
		list_add_tail(&reader->list, &log->readers);
		mutex_unlock(&log->mutex);

		file->private_data = reader;
	} else
		file->private_data = log;

	return 0;
}

/*
 * logger_release - the log's release file operation
 *
 * Note this is a total no-op in the write-only case. Keep it that way!
 */
static int logger_release(struct inode *ignored, struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;
		struct logger_log *log = reader->log;

		mutex_lock(&log->mutex);
		list_del(&reader->list);
		mutex_unlock(&log->mutex);

		kfree(reader);
	}

	return 0;
}

/*
 * logger_poll - the log's poll file operation, for poll/select/epoll
 *
 * Note we always return POLLOUT, because you can always write() to the log.
 * Note also that, strictly speaking, a return value of POLLIN does not
 * guarantee that the log is readable without blocking, as there is a small
 * chance that the writer can lap the reader in the interim between poll()
 * returning and the read() request.
 */
static unsigned int logger_poll(struct file *file, poll_table *wait)
{
	struct logger_reader *reader;
	struct logger_log *log;
	unsigned int ret = POLLOUT | POLLWRNORM;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	reader = file->private_data;
	log = reader->log;

	poll_wait(file, &log->wq, wait);

	mutex_lock(&log->mutex);
	if (!reader->r_all)
		reader->r_off = get_next_entry_by_uid(log,
			reader->r_off, current_euid());

	if (log->w_off != reader->r_off)
		ret |= POLLIN | POLLRDNORM;
	mutex_unlock(&log->mutex);

	return ret;
}

static long logger_set_version(struct logger_reader *reader, void __user *arg)
{
	int version;
	if (copy_from_user(&version, arg, sizeof(int)))
		return -EFAULT;

	if ((version < 1) || (version > 2))
		return -EINVAL;

	reader->r_ver = version;
	return 0;
}

static long logger_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct logger_log *log = file_get_log(file);
	struct logger_reader *reader;
	long ret = -EINVAL;
	void __user *argp = (void __user *) arg;

	mutex_lock(&log->mutex);

	switch (cmd) {
	case LOGGER_GET_LOG_BUF_SIZE:
		ret = log->size;
		break;
	case LOGGER_GET_LOG_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log->w_off >= reader->r_off)
			ret = log->w_off - reader->r_off;
		else
			ret = (log->size - reader->r_off) + log->w_off;
		break;
	case LOGGER_GET_NEXT_ENTRY_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;

		if (!reader->r_all)
			reader->r_off = get_next_entry_by_uid(log,
				reader->r_off, current_euid());

		if (log->w_off != reader->r_off)
			ret = get_user_hdr_len(reader->r_ver) +
				get_entry_msg_len(log, reader->r_off);
		else
			ret = 0;
		break;
	case LOGGER_FLUSH_LOG:
		if (!(file->f_mode & FMODE_WRITE)) {
			ret = -EBADF;
			break;
		}
		list_for_each_entry(reader, &log->readers, list)
			reader->r_off = log->w_off;
		log->head = log->w_off;
		log->log_info->head = log->head;
		ret = 0;
		break;
	case LOGGER_GET_VERSION:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		ret = reader->r_ver;
		break;
	case LOGGER_SET_VERSION:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		ret = logger_set_version(reader, argp);
		break;
	}

	mutex_unlock(&log->mutex);

	return ret;
}

static const struct file_operations logger_fops = {
	.owner = THIS_MODULE,
	.read = logger_read,
	.aio_write = logger_aio_write,
	.poll = logger_poll,
	.unlocked_ioctl = logger_ioctl,
	.compat_ioctl = logger_ioctl,
	.open = logger_open,
	.release = logger_release,
};

DEFINE_LOGGER_DEVICE( log_main  , ADDR_LOGCAT_MAIN  , LOGGER_LOG_MAIN  , SIZE_LOGCAT_MAIN  , ADDR_LOGGER_INFO_MAIN  , (&logger_fops) );
DEFINE_LOGGER_DEVICE( log_system, ADDR_LOGCAT_SYSTEM, LOGGER_LOG_SYSTEM, SIZE_LOGCAT_SYSTEM, ADDR_LOGGER_INFO_SYSTEM, (&logger_fops) );
DEFINE_LOGGER_DEVICE( log_events, ADDR_LOGCAT_EVENTS, LOGGER_LOG_EVENTS, SIZE_LOGCAT_EVENTS, ADDR_LOGGER_INFO_EVENTS, (&logger_fops) );
DEFINE_LOGGER_DEVICE( log_radio , ADDR_LOGCAT_RADIO , LOGGER_LOG_RADIO , SIZE_LOGCAT_RADIO , ADDR_LOGGER_INFO_RADIO , (&logger_fops) );

static struct logger_log *get_log_from_minor(int minor)
{
	if (log_main.misc.minor == minor)
		return &log_main;
	if (log_events.misc.minor == minor)
		return &log_events;
	if (log_radio.misc.minor == minor)
		return &log_radio;
	if (log_system.misc.minor == minor)
		return &log_system;
	return NULL;
}

static int __init init_log(struct logger_log *log)
{
	int ret;

	ret = misc_register(&log->misc);
	if (unlikely(ret)) {
		printk(KERN_ERR "logger: failed to register misc "
		       "device for log '%s'!\n", log->misc.name);
		return ret;
	}

	printk(KERN_INFO "logger: created %luK log '%s'\n",
	       (unsigned long) log->size >> 10, log->misc.name);

	return 0;
}

extern void init_kcj_crash_info( void );
extern void set_crash_unknown( void );
static int __init logger_init(void)
{
	int ret;

	init_kcj_crash_info();
	set_crash_unknown();

	ret = init_log(&log_main);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_events);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_radio);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_system);
	if (unlikely(ret))
		goto out;

out:
	return ret;
}

static void get_crash_time(unsigned char *buf, unsigned int bufsize)
{
	struct timespec ts_now;
	struct rtc_time tm;

	ts_now = current_kernel_time();
	rtc_time_to_tm(ts_now.tv_sec, &tm);

	sprintf( buf, "%d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
	            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	            tm.tm_hour, tm.tm_min, tm.tm_sec, ts_now.tv_nsec);
}

void set_kcj_pet_time(void){
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type*)ADDR_CONTROL_INFO;
	get_crash_time( &pPanicInfo->crash_time[0], CRASH_TIME_SIZE );
	strncat( &pPanicInfo->crash_time[0], " (LAST PET)", CRASH_TIME_SIZE-1);
	mb();
}

static unsigned char check_smem_crash_system( ram_log_info_type *plog_info )
{
	unsigned char ret = 0;

	if ( (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_KERNEL  , strlen(CRASH_SYSTEM_KERNEL ))) ||
	     (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_MODEM   , strlen(CRASH_SYSTEM_MODEM  ))) ||
	     (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_RIVA    , strlen(CRASH_SYSTEM_RIVA   ))) ||
	     (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_LPASS   , strlen(CRASH_SYSTEM_LPASS  ))) ||
	     (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_ANDROID , strlen(CRASH_SYSTEM_ANDROID))) )
	{
		ret = 1;
	}

	return ret;
}

static void set_smem_crash_system(const char *system)
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ((plog_info == NULL) || (system == NULL)) {
		return;
	}

	/* Set crash time to smem */
	memset( &plog_info->crash_time[0], '\0', CRASH_TIME_SIZE );
	get_crash_time( &plog_info->crash_time[0], CRASH_TIME_SIZE );

	if (check_smem_crash_system(plog_info) == 1) {
		return;
	}
	memcpy( &plog_info->crash_system[0], system, strlen(system) );

	mb();
}

void set_smem_crash_system_kernel(void)
{
	set_smem_crash_system(CRASH_SYSTEM_KERNEL);
}

void set_smem_crash_system_modem(void)
{
	set_smem_crash_system(CRASH_SYSTEM_MODEM);
}

void set_smem_crash_system_riva(void)
{
	set_smem_crash_system(CRASH_SYSTEM_RIVA);
}

void set_smem_crash_system_lpass(void)
{
	set_smem_crash_system(CRASH_SYSTEM_LPASS);
}

void set_smem_crash_system_android(void)
{
	set_smem_crash_system(CRASH_SYSTEM_ANDROID);
}

void set_smem_crash_system_unknown(void)
{
	set_smem_crash_system(CRASH_SYSTEM_UNKNOWN);
}

static unsigned char check_smem_crash_kind( ram_log_info_type *plog_info )
{
	unsigned char ret = 0;

	if ( (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_PANIC      , strlen(CRASH_KIND_PANIC     ))) ||
	     (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_FATAL      , strlen(CRASH_KIND_FATAL     ))) ||
	     (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_EXEPTION   , strlen(CRASH_KIND_EXEPTION  ))) ||
	     (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_WDOG_HW    , strlen(CRASH_KIND_WDOG_HW   ))) ||
	     (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_WDOG_SW    , strlen(CRASH_KIND_WDOG_SW   ))) ||
	     (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_SYS_SERVER , strlen(CRASH_KIND_SYS_SERVER))) )
	{
		ret = 1;
	}

	return ret;
}

void set_smem_crash_kind(const char *kind)
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ((plog_info == NULL) || (kind == NULL)) {
		return;
	}
	if (check_smem_crash_kind(plog_info) == 1) {
		return;
	}
	memcpy( &plog_info->crash_kind[0], kind, strlen(kind) );
	mb();
}

void set_smem_crash_kind_panic(void)
{
	set_smem_crash_kind(CRASH_KIND_PANIC);
}

void set_smem_crash_kind_fatal(void)
{
	set_smem_crash_kind(CRASH_KIND_FATAL);
}

void set_smem_crash_kind_exeption(void)
{
	set_smem_crash_kind(CRASH_KIND_EXEPTION);
}

void set_smem_crash_kind_wdog_hw(void)
{
	set_smem_crash_kind(CRASH_KIND_WDOG_HW);
}

void set_smem_crash_kind_wdog_sw(void)
{
	set_smem_crash_kind(CRASH_KIND_WDOG_SW);
}

void set_smem_crash_kind_android(void)
{
	set_smem_crash_kind(CRASH_KIND_SYS_SERVER);
}

static void set_smem_crash_kind_unknown(void)
{
	set_smem_crash_kind(CRASH_KIND_UNKNOWN);
}

/*
 * set_kcj_fixed_info_modem()
 *
 * Note: Set kcj modem fixed info to uninit ram.
 */
void set_kcj_fixed_info_modem(void)
{
        ram_log_info_type *pPanicInfo;
        ram_log_info_type *pSmemLogInfo;

        mb();
        pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
        pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

        if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
                return;
        }

        /* set modem info on smem to uninit ram */
        memcpy( &pPanicInfo->product_number[0], &pSmemLogInfo->product_number[0], PRODUCT_NUMBER_SIZE );
        memcpy( &pPanicInfo->baseband_version[0], &pSmemLogInfo->baseband_version[0], VERSION_SIZE );
        memcpy( &pPanicInfo->modem_build_date[0], &pSmemLogInfo->modem_build_date[0], BUILD_DATE_SIZE );

	/* set LinuxBanner to uninit ram */
	memcpy( &pPanicInfo->linux_banner[0], &pSmemLogInfo->linux_banner[0], LINUX_BANNER_SIZE );
        mb();
}

void set_kcj_crash_info(void)
{
	ram_log_info_type *pPanicInfo;
	ram_log_info_type *pSmemLogInfo;

	mb();
	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
		return;
	}
	if (check_smem_crash_system(pSmemLogInfo) == 0) {
		return;
	}
	if (check_smem_crash_kind(pSmemLogInfo) == 0) {
		set_smem_crash_kind_unknown();
	}

	/* set magic code to uninit_ram */
	memcpy( &pPanicInfo->magic_code[0], CRASH_MAGIC_CODE, strlen(CRASH_MAGIC_CODE) );

	/* set crash system and kind to uninit_ram */
	memcpy( &pPanicInfo->crash_system[0], &pSmemLogInfo->crash_system[0], CRASH_SYSTEM_SIZE );
	memcpy( &pPanicInfo->crash_kind[0], &pSmemLogInfo->crash_kind[0], CRASH_KIND_SIZE );

	/* set system info  to uninit_ram */
	memcpy( &pPanicInfo->linux_banner[0], &pSmemLogInfo->linux_banner[0], LINUX_BANNER_SIZE );
	memcpy( &pPanicInfo->product_number[0], &pSmemLogInfo->product_number[0], PRODUCT_NUMBER_SIZE );
	memcpy( &pPanicInfo->baseband_version[0], &pSmemLogInfo->baseband_version[0], VERSION_SIZE );
	memcpy( &pPanicInfo->modem_build_date[0], &pSmemLogInfo->modem_build_date[0], BUILD_DATE_SIZE );
	memcpy( &pPanicInfo->crash_time[0], &pSmemLogInfo->crash_time[0], CRASH_TIME_SIZE );
}

void init_kcj_crash_info( void )
{
	int               banner_len;
	ram_log_info_type *pPanicInfo;
	ram_log_info_type *pSmemLogInfo;

	mb();
	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
		return;
	}

	pPanicInfo->reserved[11] = (unsigned long)&pSmemLogInfo->reserved[0] - (unsigned long)MSM_SHARED_RAM_BASE;

	memset( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );

	/* set linux_banner to smem */
	banner_len = strlen( linux_banner );
	if ( banner_len > LINUX_BANNER_SIZE ) banner_len = LINUX_BANNER_SIZE;
	memcpy( &pSmemLogInfo->linux_banner[0], &linux_banner[0], banner_len );
}

void clear_kcj_crash_info( void )
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)ADDR_CONTROL_INFO;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memset( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );
}

void set_crash_unknown( void )
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)ADDR_CONTROL_INFO;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memcpy( &pPanicInfo->crash_system[0], CRASH_SYSTEM_UNKNOWN, strlen(CRASH_SYSTEM_UNKNOWN) );
	memcpy( &pPanicInfo->crash_kind[0]  , CRASH_KIND_UNKNOWN  , strlen(CRASH_KIND_UNKNOWN  ) );
}

device_initcall(logger_init);
