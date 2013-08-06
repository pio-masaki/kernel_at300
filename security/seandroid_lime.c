/*
 * seandroid_lime.c
 *
 * SEAndroid LIME:
 * Security Enhanced Android
 *   Light-weight Integrity measurement and Mandatory access control subsystem
 *   for Embedded devices
 *
 * Jun Kanai <jun4.kanai@toshiba.co.jp>
 * Ryuichi Koike <ryuichi.koike@toshiba.co.jp>
 *
 * based on root_plug.c
 * Copyright (C) 2002 Greg Kroah-Hartman <greg@kroah.com>
 *
 * _xx_is_valid(), _xx_encode(), _xx_realpath_from_path()
 * is ported from security/tomoyo/realpath.c in linux-2.6.32
 *
 * calc_hmac() is ported from drivers/staging/p9auth/p9auth.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/fs_struct.h>
#include <linux/mount.h>
#include <linux/mnt_namespace.h>
#include <linux/ptrace.h>
#include <linux/magic.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kobject.h>
#include <linux/genhd.h>
#include <linux/kobj_map.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <net/sock.h>
// #include "common.h"
// #include "../../fs/internal.h"

#ifdef CONFIG_SECURITY_SEALIME_DEBUGPRINT
#define PRINTK printk
#else
#define PRINTK(arg, ...)
#endif


/* This module deals with struct bus_type_private in order to get kernel internal  */
/* bus information. In common usage, the struct is private. Therefore we include a header  */
/* file from a abnormal path. */
#include "../drivers/base/base.h"

#define CONFIG_SECURITY_SEALIME_HASH_ALGORITHM "sha1"
#define TOSLSM_DIGEST_SIZE 20

#define CONFIG_SECURITY_SEALIME_WIFI_ASSOCIATE
#define SEALIME_UNLOADABLE

extern struct kset *bus_kset;

/* for SEAndroid official hooks*/
struct security_operations *lkm_secops = NULL;
/* for extra hooks. */
struct security_operations *extra_secops = NULL;


static inline bool _xx_is_valid(const unsigned char c)
{
	return c > ' ' && c < 127;
}

void _xx_warn_oom(const char *function)
{
    /* Reduce error messages. */
    static pid_t tomoyo_last_pid;
    const pid_t pid = current->pid;
    if (tomoyo_last_pid != pid) {
	printk(KERN_WARNING "ERROR: Out of memory at %s.\n",
	       function);
	tomoyo_last_pid = pid;
    }
 }


#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)))
static int _xx_encode(char *buffer, int buflen, const char *str)
{
	while (1) {
		const unsigned char c = *(unsigned char *)str++;

		if (_xx_is_valid(c)) {
			if (--buflen <= 0)
				break;
			*buffer++ = (char)c;
			if (c != '\\')
				continue;
			if (--buflen <= 0)
				break;
			*buffer++ = (char)c;
			continue;
		}
		if (!c) {
			if (--buflen <= 0)
				break;
			*buffer = '\0';
			return 0;
		}
		buflen -= 4;
		if (buflen <= 0)
			break;
		*buffer++ = '\\';
		*buffer++ = (c >> 6) + '0';
		*buffer++ = ((c >> 3) & 7) + '0';
		*buffer++ = (c & 7) + '0';
	}
	return -ENOMEM;
}
#else
char *_xx_encode(const char *str)
{
	int len = 0;
	const char *p = str;
	char *cp;
	char *cp0;

	if (!p)
		return NULL;
	while (*p) {
		const unsigned char c = *p++;
		if (c == '\\')
			len += 2;
		else if (c > ' ' && c < 127)
			len++;
		else
			len += 4;
	}
	len++;
	/* Reserve space for appending "/". */
	cp = kzalloc(len + 10, GFP_NOFS);
	if (!cp)
		return NULL;
	cp0 = cp;
	p = str;
	while (*p) {
		const unsigned char c = *p++;

		if (c == '\\') {
			*cp++ = '\\';
			*cp++ = '\\';
		} else if (c > ' ' && c < 127) {
			*cp++ = c;
		} else {
			*cp++ = '\\';
			*cp++ = (c >> 6) + '0';
			*cp++ = ((c >> 3) & 7) + '0';
			*cp++ = (c & 7) + '0';
		}
	}
	return cp0;
}
#endif


#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)))
static int _xx_realpath_from_path(struct path *path, char *newname,
				  int newname_len)
{
	struct dentry *dentry = path->dentry;
	int error = -ENOMEM;
	char *sp;

	if (!dentry || !path->mnt || !newname || newname_len <= 2048)
		return -EINVAL;
	if (dentry->d_op && dentry->d_op->d_dname) {
		/* For "socket:[\$]" and "pipe:[\$]". */
		static const int offset = 1536;
		sp = dentry->d_op->d_dname(dentry, newname + offset,
					   newname_len - offset);
	} else {
		/* Taken from d_namespace_path(). */
		struct path ns_root = { };
		struct path root;
		struct path tmp;

		read_lock(&current->fs->lock);
		root = current->fs->root;
		path_get(&root);
		read_unlock(&current->fs->lock);
		spin_lock(&vfsmount_lock);
		if (root.mnt && root.mnt->mnt_ns)
			ns_root.mnt = mntget(root.mnt->mnt_ns->root);
		if (ns_root.mnt)
			ns_root.dentry = dget(ns_root.mnt->mnt_root);
		spin_unlock(&vfsmount_lock);
		spin_lock(&dcache_lock);
		tmp = ns_root;
		sp = __d_path(path, &tmp, newname, newname_len);
		spin_unlock(&dcache_lock);
		path_put(&root);
		path_put(&ns_root);
	}
	if (IS_ERR(sp)) {
		error = PTR_ERR(sp);
	} else {
		error = _xx_encode(newname, sp - newname, sp);
	}

	/* Append trailing '/' if dentry is a directory. */
	if (!error && dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode)
	    && *newname) {
		sp = newname + strlen(newname);
		if (*(sp - 1) != '/') {
			if (sp < newname + newname_len - 4) {
				*sp++ = '/';
				*sp = '\0';
			} else {
				error = -ENOMEM;
			}
		}
	}

	return error;
}
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)))
int _xx_realpath_from_path(struct path *path, char *newname,
                           int newname_len)
{
	int error = -ENOMEM;
	struct dentry *dentry = path->dentry;
	char *sp;

	if (!dentry || !path->mnt || !newname || newname_len <= 2048)
		return -EINVAL;
	if (dentry->d_op && dentry->d_op->d_dname) {
		/* For "socket:[\$]" and "pipe:[\$]". */
		static const int offset = 1536;
		sp = dentry->d_op->d_dname(dentry, newname + offset,
		                           newname_len - offset);
	} else {
		struct path ns_root = {.mnt = NULL, .dentry = NULL};

		spin_lock(&dcache_lock);
		/* go to whatever namespace root we are under */
		sp = __d_path(path, &ns_root, newname, newname_len);
		spin_unlock(&dcache_lock);
		/* Prepend "/proc" prefix if using internal proc vfs mount. */
		if (!IS_ERR(sp) && (path->mnt->mnt_flags & MNT_INTERNAL) &&
		    (path->mnt->mnt_sb->s_magic == PROC_SUPER_MAGIC)) {
			sp -= 5;
			if (sp >= newname)
				memcpy(sp, "/proc", 5);
			else

	sp = ERR_PTR(-ENOMEM);
		}
	}
	if (IS_ERR(sp))
		error = PTR_ERR(sp);
	else
		error = _xx_encode(newname, sp - newname, sp);
	/* Append trailing '/' if dentry is a directory. */
	if (!error && dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode)
	    && *newname) {
		sp = newname + strlen(newname);
		if (*(sp - 1) != '/') {
			if (sp < newname + newname_len - 4) {
				*sp++ = '/';
				*sp = '\0';
			} else {
				error = -ENOMEM;
			}
		}
	}
	return error;
}

#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)))
static char *_xx_realpath_from_path_tmp(struct path *path)
{
    char *buf = NULL;
    char *name = NULL;
    unsigned int buf_len = PAGE_SIZE / 2;
    struct dentry *dentry = path->dentry;
    bool is_dir;
    if (!dentry)
        return NULL;
    is_dir = dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode);

    while (1) {
        struct path ns_root = { .mnt = NULL, .dentry = NULL };
        char *pos;
        buf_len <<= 1;
        kfree(buf);
        buf = kmalloc(buf_len, GFP_NOFS);
        if (!buf)
            break;
        /* Get better name for socket. */
        if (dentry->d_sb && dentry->d_sb->s_magic == SOCKFS_MAGIC) {
            struct inode *inode = dentry->d_inode;
            struct socket *sock = inode ? SOCKET_I(inode) : NULL;
            struct sock *sk = sock ? sock->sk : NULL;
            if (sk) {
                snprintf(buf, buf_len - 1, "socket:[family=%u:"
                         "type=%u:protocol=%u]", sk->sk_family,
                         sk->sk_type, sk->sk_protocol);
            } else {
                snprintf(buf, buf_len - 1, "socket:[unknown]");
            }
            name = _xx_encode(buf);
            break;
        }
        /* For "socket:[\$]" and "pipe:[\$]". */
        if (dentry->d_op && dentry->d_op->d_dname) {
            pos = dentry->d_op->d_dname(dentry, buf, buf_len - 1);
            if (IS_ERR(pos))
                continue;
            name = _xx_encode(pos);
            break;
        }
        /* If we don't have a vfsmount, we can't calculate. */
        if (!path->mnt)
            break;
        /* go to whatever namespace root we are under */
        pos = __d_path(path, &ns_root, buf, buf_len);
        /* Prepend "/proc" prefix if using internal proc vfs mount. */
        if (!IS_ERR(pos) && (path->mnt->mnt_flags & MNT_INTERNAL) &&
            (path->mnt->mnt_sb->s_magic == PROC_SUPER_MAGIC)) {
            pos -= 5;
            if (pos >= buf)
                memcpy(pos, "/proc", 5);
            else
                pos = ERR_PTR(-ENOMEM);
        }
        if (IS_ERR(pos))
            continue;
        name = _xx_encode(pos);
        break;
    }
    kfree(buf);
    if (!name)
        _xx_warn_oom(__func__);
    else if (is_dir && *name) {
        /* Append trailing '/' if dentry is a directory. */
        char *pos = name + strlen(name) - 1;
        if (*pos != '/')
            /*
             * This is OK because tomoyo_encode() reserves space
             * for appending "/".
             */
            *++pos = '/';
    }
    return name;
}

int _xx_realpath_from_path(struct path *path, char *newname, int newname_len)
{
	char *str;

	str = _xx_realpath_from_path_tmp(path);
	if(!str) {
		return -1;
	} else {
		strncpy(newname, str, newname_len-1);
		newname[newname_len-1] = '\0';
		kfree(str);
	}

	return 0;
}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0))
static char *_xx_get_dentry_path(struct dentry *dentry, char * const buffer,
				    const int buflen)
{
	char *pos = ERR_PTR(-ENOMEM);
	if (buflen >= 256) {
		pos = dentry_path_raw(dentry, buffer, buflen - 1);
		if (!IS_ERR(pos) && *pos == '/' && pos[1]) {
			struct inode *inode = dentry->d_inode;
			if (inode && S_ISDIR(inode->i_mode)) {
				buffer[buflen - 2] = '/';
				buffer[buflen - 1] = '\0';
			}
		}
	}
	return pos;
}
static char *_xx_get_absolute_path(struct path *path, char * const buffer,
				      const int buflen)
{
	char *pos = ERR_PTR(-ENOMEM);
	if (buflen >= 256) {
		/* go to whatever namespace root we are under */
		pos = d_absolute_path(path, buffer, buflen - 1);
		if (!IS_ERR(pos) && *pos == '/' && pos[1]) {
			struct inode *inode = path->dentry->d_inode;
			if (inode && S_ISDIR(inode->i_mode)) {
				buffer[buflen - 2] = '/';
				buffer[buflen - 1] = '\0';
			}
		}
	}
	return pos;
}
static char *_xx_get_local_path(struct dentry *dentry, char * const buffer,
				   const int buflen)
{
	struct super_block *sb = dentry->d_sb;
	char *pos = _xx_get_dentry_path(dentry, buffer, buflen);
	if (IS_ERR(pos))
		return pos;
	/* Convert from $PID to self if $PID is current thread. */
	if (sb->s_magic == PROC_SUPER_MAGIC && *pos == '/') {
		char *ep;
		const pid_t pid = (pid_t) simple_strtoul(pos + 1, &ep, 10);
		if (*ep == '/' && pid && pid ==
		    task_tgid_nr_ns(current, sb->s_fs_info)) {
			pos = ep - 5;
			if (pos < buffer)
				goto out;
			memmove(pos, "/self", 5);
		}
		goto prepend_filesystem_name;
	}
	/* Use filesystem name for unnamed devices. */
	if (!MAJOR(sb->s_dev))
		goto prepend_filesystem_name;
	{
		struct inode *inode = sb->s_root->d_inode;
		/*
		 * Use filesystem name if filesystem does not support rename()
		 * operation.
		 */
		if (inode->i_op && !inode->i_op->rename)
			goto prepend_filesystem_name;
	}
	/* Prepend device name. */
	{
		char name[64];
		int name_len;
		const dev_t dev = sb->s_dev;
		name[sizeof(name) - 1] = '\0';
		snprintf(name, sizeof(name) - 1, "dev(%u,%u):", MAJOR(dev),
			 MINOR(dev));
		name_len = strlen(name);
		pos -= name_len;
		if (pos < buffer)
			goto out;
		memmove(pos, name, name_len);
		return pos;
	}
	/* Prepend filesystem name. */
prepend_filesystem_name:
	{
		const char *name = sb->s_type->name;
		const int name_len = strlen(name);
		pos -= name_len + 1;
		if (pos < buffer)
			goto out;
		memmove(pos, name, name_len);
		pos[name_len] = ':';
	}
	return pos;
out:
	return ERR_PTR(-ENOMEM);
}
static char *_xx_get_socket_name(struct path *path, char * const buffer,
				    const int buflen)
{
	struct inode *inode = path->dentry->d_inode;
	struct socket *sock = inode ? SOCKET_I(inode) : NULL;
	struct sock *sk = sock ? sock->sk : NULL;
	if (sk) {
		snprintf(buffer, buflen, "socket:[family=%u:type=%u:"
			 "protocol=%u]", sk->sk_family, sk->sk_type,
			 sk->sk_protocol);
	} else {
		snprintf(buffer, buflen, "socket:[unknown]");
	}
	return buffer;
}
static char *_xx_realpath_from_path_tmp(struct path *path)
{
	char *buf = NULL;
	char *name = NULL;
	unsigned int buf_len = PAGE_SIZE / 2;
	struct dentry *dentry = path->dentry;
	struct super_block *sb;
	if (!dentry)
		return NULL;
	sb = dentry->d_sb;
	while (1) {
		char *pos;
		struct inode *inode;
		buf_len <<= 1;
		kfree(buf);
		buf = kmalloc(buf_len, GFP_NOFS);
		if (!buf)
			break;
		/* To make sure that pos is '\0' terminated. */
		buf[buf_len - 1] = '\0';
		/* Get better name for socket. */
		if (sb->s_magic == SOCKFS_MAGIC) {
			pos = _xx_get_socket_name(path, buf, buf_len - 1);
			goto encode;
		}
		/* For "pipe:[\$]". */
		if (dentry->d_op && dentry->d_op->d_dname) {
			pos = dentry->d_op->d_dname(dentry, buf, buf_len - 1);
			goto encode;
		}
		inode = sb->s_root->d_inode;
		/*
		 * Get local name for filesystems without rename() operation
		 * or dentry without vfsmount.
		 */
		if (!path->mnt || (inode->i_op && !inode->i_op->rename))
			pos = _xx_get_local_path(path->dentry, buf,
						    buf_len - 1);
		/* Get absolute name for the rest. */
		else {
			pos = _xx_get_absolute_path(path, buf, buf_len - 1);
			/*
			 * Fall back to local name if absolute name is not
			 * available.
			 */
			if (pos == ERR_PTR(-EINVAL))
				pos = _xx_get_local_path(path->dentry, buf,
							    buf_len - 1);
		}
encode:
		if (IS_ERR(pos))
			continue;
		name = _xx_encode(pos);
		break;
	}
	kfree(buf);
	if (!name)
		_xx_warn_oom(__func__);
	return name;
}
int _xx_realpath_from_path(struct path *path, char *newname, int newname_len)
{
	char *str;

	str = _xx_realpath_from_path_tmp(path);
	if(!str) {
		return -1;
	} else {
		strncpy(newname, str, newname_len-1);
		newname[newname_len-1] = '\0';
		kfree(str);
	}

	return 0;
}
#endif


EXPORT_SYMBOL(_xx_realpath_from_path);


static char *calc_hmac(char *plain_text, unsigned int plain_text_size,
		      char *key, unsigned int key_size)
{
	struct scatterlist sg;
	char *result;
	struct crypto_hash *tfm;
	struct hash_desc desc;
	int ret;

	tfm = crypto_alloc_hash("hmac(sha1)", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		PRINTK(KERN_ERR
		       "failed to load transform for hmac(sha1): %ld\n",
		       PTR_ERR(tfm));
		return NULL;
	}

	desc.tfm = tfm;
	desc.flags = 0;

	result = kzalloc(TOSLSM_DIGEST_SIZE, GFP_KERNEL);
	if (!result) {
		PRINTK(KERN_ERR "out of memory!\n");
		goto out;
	}

	sg_set_buf(&sg, plain_text, plain_text_size);

	ret = crypto_hash_setkey(tfm, key, key_size);
	if (ret) {
		PRINTK(KERN_ERR "setkey() failed ret=%d\n", ret);
		kfree(result);
		result = NULL;
		goto out;
	}

	ret = crypto_hash_digest(&desc, &sg, plain_text_size, result);
	if (ret) {
		PRINTK(KERN_ERR "digest() failed ret=%d\n", ret);
		kfree(result);
		result = NULL;
		goto out;
	}

out:
	crypto_free_hash(tfm);
	return result;
}
EXPORT_SYMBOL(calc_hmac);

/* device dependent ? */
#define DEVICE_TYPE "usb_device"
#define DEVICE_DRIVER_NAME "usb"
#define MMC_DRIVER_NAME "mmcblk"

enum inter_bus {USB, MMC, NONE};

enum req_stat {
	SEARCH_MN_NUM,
	SEARCH_MN_FOUND,
	SEARCH_USB_DESCRIPTOR,
	SEARCH_MMC_DESCRIPTOR,
	SEARCH_FOUND,
};

struct device_search_request {
	int major;
	int minor;
	enum req_stat stat;
	enum inter_bus bus;
	struct device *dev;
};

struct device_search_response {
	enum inter_bus bus;
	struct device *dev;
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
#define to_bus(obj) container_of(obj, struct subsys_private, subsys.kobj)
#else
#define to_bus(obj) container_of(obj, struct bus_type_private, subsys.kobj)
#endif

static int is_usb_device_root(struct device *dev)
{
	if(!dev->driver) return 0;
	if(!dev->type)   return 0;


	if((!strcmp(dev->type->name, DEVICE_TYPE)) &&
		(!strcmp(dev->driver->name, DEVICE_DRIVER_NAME))) {
		return 1;
	}
	return 0;
}

static int is_mmc_device_root(struct device *dev)
{
	if(!dev->driver) return 0;

	if(!strcmp(dev->driver->name, MMC_DRIVER_NAME)) {
		return 1;
	}
	return 0;
}

static int match_mn_num(struct device *dev, void *data)
{
	struct device_search_request *req = (struct device_search_request *)data;

	if(req->bus == NONE) {
		 return 0;
	}

	if(req->stat == SEARCH_FOUND) {
		return 0;
	}

	if(req->stat == SEARCH_MN_NUM) {
		if( (req->major == MAJOR(dev->devt)) && (req->minor == MINOR(dev->devt)) ) {
			req->dev = dev;
			if(req->bus == USB) {req->stat = SEARCH_USB_DESCRIPTOR; }
			if(req->bus == MMC) {req->stat = SEARCH_MMC_DESCRIPTOR; }
		}
	}

	device_for_each_child(dev, data, match_mn_num);

	if(req->stat == SEARCH_USB_DESCRIPTOR && is_usb_device_root(dev)) {
		req->dev = dev;
		req->stat = SEARCH_FOUND;
	}
	if(req->stat == SEARCH_MMC_DESCRIPTOR && is_mmc_device_root(dev)) {
		req->dev = dev;
		req->stat = SEARCH_FOUND;
	}

	return 0;
}

/* search device by using major number no and minor no  */
static int device_find_mn_num(struct device_search_request *req, struct device_search_response *res)
{
	struct list_head *p;

	kset_get(bus_kset);
	list_for_each(p, &(bus_kset->list)) {
		struct kobject *bus_kobj = container_of(p, struct kobject, entry);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
		struct subsys_private *pbus = to_bus(bus_kobj);
#else
		struct bus_type_private *pbus = to_bus(bus_kobj);
#endif

		req->stat = SEARCH_MN_NUM;
		req->bus = NONE;

		/* PRINTK("[BUS]=%s\n", bus_kobj->name); */
		if(!strcmp(bus_kobj->name, "usb")) {
			req->bus = USB;
		} else if(!strcmp(bus_kobj->name, "mmc")) {
			req->bus = MMC;
		} else {
			req->bus = NONE;
		}

		bus_for_each_dev(pbus->bus, NULL, req, match_mn_num);

		if(req->stat == SEARCH_FOUND) {
			res->bus = req->bus;
			res->dev = req->dev;
			return 1;		/* found */
		}
	}
	kset_put(bus_kset);
	return 0;					/* not found */
}
EXPORT_SYMBOL(device_find_mn_num);


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#define PTRACE_ACCESS_CHECK ptrace_access_check
#else
#define PTRACE_ACCESS_CHECK ptrace_may_access
#endif
static int sealime_ptrace_access_check(struct task_struct *child,
				       unsigned int mode)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->PTRACE_ACCESS_CHECK (child, mode);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->PTRACE_ACCESS_CHECK) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: ptrace_access_check\n");
		r = extra_secops->PTRACE_ACCESS_CHECK (child, mode);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: ptrace_access_check = %d\n", r);
	}

	return r;
}

static int sealime_ptrace_traceme(struct task_struct *parent)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->ptrace_traceme(parent);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->ptrace_traceme) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: ptrace_traceme\n");
		r = extra_secops->ptrace_traceme(parent);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: ptrace_traceme = %d\n", r);
	}

	return r;
}

static int sealime_sb_mount(char *dev_name, struct path *path,
			    char *type, unsigned long flags, void *data)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->sb_mount(dev_name, path, type, flags, data);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->sb_mount) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: sb_mount\n");
		r = extra_secops->sb_mount(dev_name, path, type, flags, data);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: sb_mount = %d\n", r);
	}

	return r;
}

static int sealime_sb_umount(struct vfsmount *mnt, int flags)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->sb_umount(mnt, flags);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->sb_umount) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: sb_umount\n");
		r = extra_secops->sb_umount(mnt, flags);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: sb_umount = %d\n", r);
	}

	return r;
}

static int sealime_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->sb_pivotroot(old_path, new_path);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->sb_pivotroot) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: pivotroot\n");
		r = extra_secops->sb_pivotroot(old_path, new_path);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: pivotroot = %d\n", r);
	}

	return r;
}

static int sealime_path_chroot(struct path *path)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_chroot(path);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_chroot) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_chroot\n");
		r = extra_secops->path_chroot(path);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_chroot = %d\n", r);
	}

	return r;
}

static int sealime_file_permission(struct file *file, int mask)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->file_permission(file, mask);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->file_permission) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: file_permission\n");
		r = extra_secops->file_permission(file, mask);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: file_permission = %d\n", r);
	}

	return r;
}

static int sealime_bprm_secureexec(struct linux_binprm *bprm)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->bprm_secureexec(bprm);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->bprm_secureexec) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: bprm_secureexec\n");
		r = extra_secops->bprm_secureexec(bprm);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: bprm_secureexec = %d\n", r);
	}

	return r;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
static int sealime_path_truncate(struct path *path)
#else
static int sealime_path_truncate(struct path *path, loff_t length, unsigned int time_attrs)
#endif
{
	int r = 0;

	if (lkm_secops) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
		r = lkm_secops->path_truncate(path);
#else
		r = lkm_secops->path_truncate(path, length, time_attrs);
#endif
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_truncate) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_truncate\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
		r = extra_secops->path_truncate(path);
#else
		r = extra_secops->path_truncate(path, length, time_attrs);
#endif
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_truncate = %d\n", r);
	}

	return r;
}



static int sealime_path_mknod(struct path *path, struct dentry *dentry,
			      int mode, unsigned int dev)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_mknod(path, dentry, mode, dev);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_mknod) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_mkmod\n");
		r = extra_secops->path_mknod(path, dentry, mode, dev);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_mkmod = %d\n", r);
	}

	return r;
}

static int sealime_path_unlink(struct path *path, struct dentry *dentry)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_unlink(path, dentry);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_unlink) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_unlink\n");
		r = extra_secops->path_unlink(path, dentry);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_unlink = %d\n", r);
	}

	return r;
}

static int sealime_path_rename(struct path *old_dir, struct dentry *old_dentry,
			       struct path *new_dir, struct dentry *new_dentry)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_rename(old_dir, old_dentry, new_dir, new_dentry);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_rename) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_rename\n");
		r = extra_secops->path_rename(old_dir, old_dentry, new_dir, new_dentry);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_rename = %d\n", r);
	}

	return r;
}

static int sealime_task_create(unsigned long clone_flags)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->task_create(clone_flags);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->task_create) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: task_create\n");
		r = extra_secops->task_create(clone_flags);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: task_create = %d\n", r);
	}

	return r;
}

static int sealime_init_module(const char *image, unsigned long len)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->init_module(image, len);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->init_module) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: init_module\n");
		r = extra_secops->init_module(image, len);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: init_module = %d\n", r);
	}

	return r;
}

#ifdef CONFIG_SECURITY_SEALIME_WIFI_ASSOCIATE
static int sealime_wifi_associate(const char *ifname, const char *bssid, const char *ssid, int ssid_len)
{
	int r = 0;
	if (lkm_secops) {
		r = lkm_secops->wifi_associate(ifname, bssid, ssid, ssid_len);
	}

	if (r != 0) {
		return r;
	}

	if (extra_secops && extra_secops->wifi_associate) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: wifi_associate\n");
		r = extra_secops->wifi_associate(ifname, bssid, ssid, ssid_len);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: wifi_associate = %d\n", r);
	}

	return r;
}
#endif
static int sealime_task_prctl(int option, unsigned long arg2,
			      unsigned long arg3, unsigned long arg4,
			      unsigned long arg5)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->task_prctl(option, arg2, arg3, arg4,arg5);
	} else {
		r = cap_task_prctl(option, arg2, arg3, arg3, arg5);
	}

	if(r == -EPERM) {
		return r;
	}

	if(extra_secops && extra_secops->task_prctl) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: dentry_open\n");
		r = extra_secops->task_prctl(option, arg2, arg3, arg4,arg5);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: dentry_open = %d\n", r);
	}

	return r;


/* 	if (lkm_secops) return lkm_secops->task_prctl(option, arg2, arg3, arg4, arg5); */
/* 	return cap_task_prctl(option, arg2, arg3, arg3, arg5); */
}

static int sealime_dentry_open(struct file *file, const struct cred *cred)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->dentry_open(file, cred);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->dentry_open) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: dentry_open\n");
		r = extra_secops->dentry_open(file, cred);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: dentry_open = %d\n", r);
	}

	return r;
}

static int sealime_path_mkdir(struct path *dir, struct dentry *dentry, int mode)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_mkdir(dir, dentry, mode);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_mkdir) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_mkdir\n");
		r = extra_secops->path_mkdir(dir, dentry, mode);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_mkdir = %d\n", r);
	}

	return r;
}

static int sealime_path_rmdir(struct path *dir, struct dentry *dentry)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_rmdir(dir, dentry);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_rmdir) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_rmdir\n");
		r = extra_secops->path_rmdir(dir, dentry);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_rmdir = %d\n", r);
	}

	return r;
}

static int sealime_path_symlink(struct path *dir, struct dentry *dentry, const char *old_name)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_symlink(dir, dentry, old_name);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_symlink) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_symlink\n");
		r = extra_secops->path_symlink(dir, dentry, old_name);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_symlink = %d\n", r);
	}

	return 0;
}

static int sealime_path_link(struct dentry *old_dentry, struct path *new_dir, struct dentry *new_dentry)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->path_link(old_dentry, new_dir, new_dentry);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->path_link) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: path_link\n");
		r = extra_secops->path_link(old_dentry, new_dir, new_dentry);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: path_link = %d\n", r);
	}

	return 0;
}

static int sealime_file_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->file_ioctl(file, cmd, arg);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->file_ioctl) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: file_ioctl\n");
		r = extra_secops->file_ioctl(file, cmd, arg);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: file_ioctl = %d\n", r);
	}

	return r;
}

static int sealime_socket_connect (struct socket *sock,
                                   struct sockaddr *address, int addrlen)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->socket_connect (sock, address, addrlen);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->socket_connect) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: socket_connect\n");
		r = extra_secops->socket_connect (sock, address, addrlen);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: socket_connect = %d\n", r);
	}

	return r;
}

static int sealime_socket_accept (struct socket *sock, struct socket *newsock)
{
	int r = 0;

	if (lkm_secops) {
		r = lkm_secops->socket_accept(sock, newsock);
	}

	if(r != 0) {
		return r;
	}

	if(extra_secops && extra_secops->socket_accept) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] run extra hook: socket_accept\n");
		r = extra_secops->socket_accept(sock, newsock);
		PRINTK(KERN_INFO "[SEAndroid_Lime] return extra hook: socket_accept = %d\n", r);
	}

	return r;
}

static struct security_operations sealime_security_ops = {
	.PTRACE_ACCESS_CHECK = sealime_ptrace_access_check,
	.ptrace_traceme = sealime_ptrace_traceme,
	.sb_mount = sealime_sb_mount,
	.sb_umount = sealime_sb_umount,
	.sb_pivotroot = sealime_sb_pivotroot,
	.file_permission = sealime_file_permission,
	.bprm_secureexec = sealime_bprm_secureexec,
	.path_truncate = sealime_path_truncate,
	.path_mknod = sealime_path_mknod,
	.path_unlink = sealime_path_unlink,
	.path_rename = sealime_path_rename,
	.path_symlink = sealime_path_symlink,
	.path_link = sealime_path_link,
	.task_create = sealime_task_create,
	.path_chroot = sealime_path_chroot,
	.task_prctl = sealime_task_prctl,
	.dentry_open = sealime_dentry_open,
	.init_module = sealime_init_module,
	.path_mkdir = sealime_path_mkdir,
	.path_rmdir = sealime_path_rmdir,
	.file_ioctl = sealime_file_ioctl,
	.socket_connect = sealime_socket_connect,
	.socket_accept = sealime_socket_accept,
#ifdef CONFIG_SECURITY_SEALIME_WIFI_ASSOCIATE
	.wifi_associate = sealime_wifi_associate,
#endif
};

static int __init sealime_init(void)
{
	if (register_security(&sealime_security_ops)) {
		PRINTK(KERN_INFO "[SEAndroid_Lime] Failure registering LSM\n");
		return -EINVAL;
	}
	PRINTK(KERN_INFO "[SEAndroid_Lime] LSM module initialized\n");

	return 0;
}

static int register_sealime(struct security_operations *sec_ops) {
#ifndef SEALIME_UNLOADABLE
	if (lkm_secops == NULL) {
#endif
		lkm_secops = sec_ops;
		PRINTK(KERN_INFO "[SEAndroid_LIME] allow: Sealime LKM is registered!\n");
#ifndef SEALIME_UNLOADABLE
	} else {
		PRINTK(KERN_INFO "[SEAndroid_LIME] reject: Sealime LKM is already registered!\n");
	}
#endif
	return 0;
}

static int register_extra_hook(struct security_operations *sec_ops) {
#ifdef SEALIME_UNLOADABLE
	if(sec_ops == NULL) {
		extra_secops = NULL;
		return 0;
	}
#endif

	if (sec_ops != NULL) {
		extra_secops = sec_ops;
		PRINTK(KERN_INFO "[SEAndroid_LIME] allow: Extra hook is registered!\n");
	}

	return 0;
}

EXPORT_SYMBOL(register_sealime);
EXPORT_SYMBOL(register_extra_hook);
EXPORT_SYMBOL(__ptrace_unlink);
EXPORT_SYMBOL(cap_task_prctl);


security_initcall(sealime_init);
