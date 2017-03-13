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
#ifndef _RTDM_I2C_DEVICE_H
#define _RTDM_I2C_DEVICE_H

#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <rtdm/driver.h>
#include <rtdm/uapi/i2c.h>

struct class;
struct rtdm_i2c_master;

struct rtdm_i2c_remote_slave {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
	int irq;			/* irq issued by device		*/
	struct rtdm_device dev;
	struct list_head next;
	struct rtdm_i2c_config config;
	struct rtdm_i2c_master *master;
	atomic_t mmap_refs;
	struct mutex ctl_lock;
};

static inline struct device *
slave_to_kdev(struct rtdm_i2c_remote_slave *slave)
{
	return rtdm_dev_to_kdev(&slave->dev);
}

int rtdm_i2c_add_remote_slave(struct rtdm_i2c_remote_slave *slave,
        struct rtdm_i2c_master *spim, struct i2c_client *i2c);

void rtdm_i2c_remove_remote_slave(struct rtdm_i2c_remote_slave *slave);

#endif /* !_RTDM_I2C_DEVICE_H */
