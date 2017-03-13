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
#ifndef _RTDM_I2C_MASTER_H
#define _RTDM_I2C_MASTER_H

#include <rtdm/driver.h>
#include <rtdm/uapi/i2c.h>
//#include "i2c-device.h"

struct class;
struct device_node;
struct rtdm_i2c_master;
struct i2c_adapter;

struct rtdm_i2c_master_ops {
	int (*open)(struct rtdm_i2c_master *m);
	void (*close)(struct rtdm_i2c_master *m);
	int (*configure)(struct rtdm_i2c_master *m);
	int (*transfer)(struct rtdm_i2c_master *m, struct i2c_msg *msgs, int num);
	ssize_t (*write)(struct rtdm_i2c_master *m, const void *tx, size_t len);
	ssize_t (*read)(struct rtdm_i2c_master *m, void *rx, size_t len);
};

struct rtdm_i2c_master {
	int subclass;
	const struct rtdm_i2c_master_ops *ops;
	struct i2c_adapter *kadapter;
    struct rtdm_device dev;
	struct rtdm_i2c_config config;
	struct {	/* Internal */
		struct rtdm_driver driver;
		struct class *devclass;
		char *classname;
		struct list_head next;
		rtdm_lock_t lock;
		rtdm_mutex_t bus_lock;
	};
};

#define rtdm_i2c_alloc_master(__dev, __type, __mptr)			\
	__rtdm_i2c_alloc_master(__dev, sizeof(__type),			\
				offsetof(__type, __mptr))		\

struct rtdm_i2c_master *
__rtdm_i2c_alloc_master(struct device *dev, size_t size, int off);

int __rtdm_i2c_setup_driver(struct rtdm_i2c_master *master);

int rtdm_i2c_add_master(struct rtdm_i2c_master *master);

void rtdm_i2c_remove_master(struct rtdm_i2c_master *master);

#endif /* !_RTDM_I2C_MASTER_H */
