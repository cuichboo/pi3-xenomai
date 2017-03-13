/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "i2c-master.h"

static inline
struct device *to_kdev(struct rtdm_i2c_master *master)
{
	return rtdm_dev_to_kdev(&master->dev);
}

static inline struct rtdm_i2c_master *fd_to_master(struct rtdm_fd *fd)
{
	struct rtdm_device *dev = rtdm_fd_device(fd);

	return container_of(dev, struct rtdm_i2c_master, dev);
}

static int update_master_config(struct rtdm_i2c_master *master,
			       struct rtdm_i2c_config *config)
{
	struct rtdm_i2c_config old_config;
	int ret;

	rtdm_mutex_lock(&master->bus_lock);

	memcpy(&old_config, &master->config, sizeof(old_config));
	memcpy(&master->config, config, sizeof(*config));
	ret = master->ops->configure(master);
	if (ret) {
        memcpy(&master->config, &old_config, sizeof(old_config));
		rtdm_mutex_unlock(&master->bus_lock);
		return ret;
	}

	rtdm_mutex_unlock(&master->bus_lock);
	
    /*
	dev_info(to_kdev(master),
		 "configured addr 0x%02x, flag 0x%02x, rate = %dHz max\n",
         config->addr,
         config->flags,
         config->rate);
         */
	
	return 0;
}

static int i2c_master_open(struct rtdm_fd *fd, int oflags)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);

	if (master->ops->open)
		return master->ops->open(master);
		
	return 0;
}

static void i2c_master_close(struct rtdm_fd *fd)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);
	//rtdm_lockctx_t c;

	if (master->ops->close)
		master->ops->close(master);
}

static void *rtdm_memdup_user(struct rtdm_fd *fd, const void *src, int len)
{
	void *p = xnmalloc(len);

    if (!p) {
		return ERR_PTR(-ENOMEM);
    }

    if (rtdm_safe_copy_from_user(fd, p, src, len)) {
		xnfree(p);
		return ERR_PTR(-EFAULT);
    }

    return p;
}

static int i2c_master_ioctl_rt(struct rtdm_fd *fd,
			       unsigned int request, void *arg)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);
	struct rtdm_i2c_config config;
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, ret;

	switch (request) {
    case I2C_RTIOC_SET_CONFIG:
		ret = rtdm_safe_copy_from_user(fd, &config,
					       arg, sizeof(config));
		if (ret == 0)
			ret = update_master_config(master, &config);
		break;
    
    case I2C_RTIOC_GET_CONFIG:
		rtdm_mutex_lock(&master->bus_lock);
		config = master->config;
		rtdm_mutex_unlock(&master->bus_lock);
		ret = rtdm_safe_copy_to_user(fd, arg,
					     &config, sizeof(config));
        break;

	case I2C_RTIOC_TRANSFER:
		ret = rtdm_safe_copy_from_user(fd, &rdwr_arg,
                            arg, sizeof(rdwr_arg));
        if (ret) {
            return -EFAULT;
        }

        if (rdwr_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS) {
            printk("excced max num: %u\n", rdwr_arg.nmsgs);
            return -EINVAL;
        }

        rdwr_pa = rtdm_memdup_user(fd, rdwr_arg.msgs,
                      rdwr_arg.nmsgs * sizeof(struct i2c_msg));
        if (IS_ERR(rdwr_pa))
            return PTR_ERR(rdwr_pa);

        data_ptrs = xnmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *));
        if (data_ptrs == NULL) {
            xnfree(rdwr_pa);
            return -ENOMEM;
        }

        ret = 0;
        for (i = 0; i < rdwr_arg.nmsgs; i++) {
            /* Limit the size of the message to a sane amount */
            if (rdwr_pa[i].len > 8192) {
                printk("limit of size\n");
                ret = -EINVAL;
                break;
            }

            data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
            rdwr_pa[i].buf = rtdm_memdup_user(fd, data_ptrs[i], rdwr_pa[i].len);
            if (IS_ERR(rdwr_pa[i].buf)) {
                ret = PTR_ERR(rdwr_pa[i].buf);
                break;
            }

            /*
             * If the message length is received from the slave (similar
             * to SMBus block read), we must ensure that the buffer will
             * be large enough to cope with a message length of
             * I2C_SMBUS_BLOCK_MAX as this is the maximum underlying bus
             * drivers allow. The first byte in the buffer must be
             * pre-filled with the number of extra bytes, which must be
             * at least one to hold the message length, but can be
             * greater (for example to account for a checksum byte at
             * the end of the message.)
             */
            if (rdwr_pa[i].flags & I2C_M_RECV_LEN) {
                if (!(rdwr_pa[i].flags & I2C_M_RD) ||
                    rdwr_pa[i].buf[0] < 1 ||
                    rdwr_pa[i].len < rdwr_pa[i].buf[0] +
                             I2C_SMBUS_BLOCK_MAX) {
                    ret = -EINVAL;
                    break;
                }

                rdwr_pa[i].len = rdwr_pa[i].buf[0];
            }
        }
        if (ret < 0) {
            int j;
            for (j = 0; j < i; ++j)
                xnfree(rdwr_pa[j].buf);
            xnfree(data_ptrs);
            xnfree(rdwr_pa);
            return ret;
        }

        rtdm_mutex_lock(&master->bus_lock);
        ret = master->ops->transfer(master, rdwr_pa, rdwr_arg.nmsgs);
        rtdm_mutex_unlock(&master->bus_lock);
        while (i-- > 0) {
            if (ret >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
                if (rtdm_safe_copy_to_user(fd, data_ptrs[i], rdwr_pa[i].buf,
                         rdwr_pa[i].len))
                    ret = -EFAULT;
            }
            xnfree(rdwr_pa[i].buf);
        }
        xnfree(data_ptrs);
        xnfree(rdwr_pa);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static int i2c_master_ioctl_nrt(struct rtdm_fd *fd,
				unsigned int request, void *arg)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);
	int ret;

	switch (request) {
    case I2C_RTIOC_SET_CONFIG:
        printk("nrt set config\n");
        break;
    case I2C_RTIOC_GET_CONFIG:
        printk("nrt get config\n");
        break;
	case I2C_RTIOC_TRANSFER:
        printk("nrt transfer\n");
        break;
	default:
        (void)master;
	}
    
    ret = -EINVAL;

	return ret;
}

static ssize_t i2c_master_read_rt(struct rtdm_fd *fd,
				  void __user *u_buf, size_t len)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);
	void *rx;
	int ret;

	if (len == 0)
		return 0;

	rx = xnmalloc(len);
	if (rx == NULL)
		return -ENOMEM;

	rtdm_mutex_lock(&master->bus_lock);
    ret = master->ops->read(master, rx, len);
	rtdm_mutex_unlock(&master->bus_lock);
	if (ret > 0) {
		ret = rtdm_safe_copy_to_user(fd, u_buf, rx, len);
#ifdef DEBUG_RTDM_I2C
        int i=0;
        printk("rtdm i2c read: ");
        for (i=0; i<len; i++) {
            printk("%02x ", ((uint8_t *)rx)[i]);
        }
        printk("\n");
#endif
    }

	xnfree(rx);
	
	return ret;
}

static ssize_t i2c_master_write_rt(struct rtdm_fd *fd,
				   const void __user *u_buf, size_t len)
{
	struct rtdm_i2c_master *master = fd_to_master(fd);
	void *tx;
	int ret;
    
	if (len == 0)
		return 0;

	tx = xnmalloc(len);
	if (tx == NULL)
		return -ENOMEM;

	ret = rtdm_safe_copy_from_user(fd, tx, u_buf, len);
	if (ret == 0) {
#ifdef DEBUG_RTDM_I2C
        int i=0;
        printk("rtdm i2c write: ");
        for (i=0; i<len; i++) {
            printk("%02x ", ((uint8_t *)tx)[i]);
        }
        printk("\n");
#endif

		rtdm_mutex_lock(&master->bus_lock);
        ret = master->ops->write(master, tx, len);
		rtdm_mutex_unlock(&master->bus_lock);
	}

	xnfree(tx);

	return ret;
}

static char *i2c_master_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "rtdm/%s", dev_name(dev));
}

struct rtdm_i2c_master *
__rtdm_i2c_alloc_master(struct device *dev, size_t size, int off)
{
	struct rtdm_i2c_master *master;
	struct i2c_adapter *kadapter;

	kadapter = kzalloc(size + sizeof(*kadapter), GFP_KERNEL);
	if (!kadapter)
		return NULL;
	
	master = (void *)(kadapter + 1) + off;
	master->kadapter = kadapter;
	i2c_set_adapdata(kadapter, master);

	return master;
}
EXPORT_SYMBOL_GPL(__rtdm_i2c_alloc_master);

int __rtdm_i2c_setup_driver(struct rtdm_i2c_master *master)
{
	master->classname = kstrdup(
		dev_name(&master->kadapter->dev), GFP_KERNEL);
	master->devclass = class_create(THIS_MODULE,
		master->classname);
	if (IS_ERR(master->devclass)) {
		kfree(master->classname);
		printk(XENO_ERR "cannot create sysfs class\n");
		return PTR_ERR(master->devclass);
	}

	master->devclass->devnode = i2c_master_devnode;

	master->driver.profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(rtdm_i2c_master,
				  RTDM_CLASS_I2C,
				  master->subclass,
				  0);
	master->driver.device_flags = RTDM_NAMED_DEVICE;
	master->driver.base_minor = 0;
	master->driver.device_count = 256;
	master->driver.context_size = 0;
	master->driver.ops = (struct rtdm_fd_ops){
		.open		=	i2c_master_open,
		.close		=	i2c_master_close,
		.read_rt	=	i2c_master_read_rt,
		.write_rt	=	i2c_master_write_rt,
		.ioctl_rt	=	i2c_master_ioctl_rt,
		.ioctl_nrt	=	i2c_master_ioctl_nrt,
	};
	
	rtdm_drv_set_sysclass(&master->driver, master->devclass);

	rtdm_lock_init(&master->lock);
	rtdm_mutex_init(&master->bus_lock);

	return 0;
}

static int i2c_transfer_one_unimp(struct i2c_adapter *adap,
	struct i2c_msg *msgs, int num)
{
	return -ENODEV;
}

static struct i2c_algorithm i2c_algo_unimp = {
	.master_xfer = i2c_transfer_one_unimp,
};

int rtdm_i2c_add_master(struct rtdm_i2c_master *master)
{
	struct i2c_adapter *kadapter = master->kadapter;
	struct rtdm_device *dev;
	int ret;

	kadapter->algo = &i2c_algo_unimp;
	master->devclass = NULL;
    
    dev = &master->dev;
    dev->driver = &master->driver;
	dev->label = kasprintf(GFP_KERNEL, "i2c-%d",
                   kadapter->nr);
    ret = __rtdm_i2c_setup_driver(master);
    if (ret)
        return ret;

	ret = rtdm_dev_register(dev);
	if (ret)
        return ret;

	/*
	 * Add the core SPI driver, devices on the bus will be
	 * enumerated, handed to i2c_device_probe().
	 */
	return i2c_add_numbered_adapter(kadapter);
}
EXPORT_SYMBOL_GPL(rtdm_i2c_add_master);

void rtdm_i2c_remove_master(struct rtdm_i2c_master *master)
{
	struct class *class = master->devclass;
	char *classname = master->classname;
	
	rtdm_mutex_destroy(&master->bus_lock);
	i2c_del_adapter(master->kadapter);
	rtdm_drv_set_sysclass(&master->driver, NULL);
	class_destroy(class);
	kfree(classname);
}
EXPORT_SYMBOL_GPL(rtdm_i2c_remove_master);

MODULE_LICENSE("GPL");
