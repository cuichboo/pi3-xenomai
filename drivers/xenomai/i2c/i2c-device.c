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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include "i2c-master.h"

int rtdm_i2c_add_remote_slave(struct rtdm_i2c_remote_slave *slave,
			      struct rtdm_i2c_master *master,
			      struct i2c_client *i2c)
{
	struct i2c_adapter *kadapter = master->kadapter;
	struct rtdm_device *dev;
	rtdm_lockctx_t c;
	int ret;

	memset(slave, 0, sizeof(*slave));
	slave->config.addr = i2c->addr;
	slave->master = master;
	
	dev = &slave->dev;
	dev->driver = &master->driver;
	dev->label = kasprintf(GFP_KERNEL, "%s/slave%d.%%d",
			       dev_name(&kadapter->dev),
                   slave->config.addr
                   );

    if (dev->label == NULL)
		return -ENOMEM;

	mutex_init(&slave->ctl_lock);

	dev->device_data = master;
	ret = rtdm_dev_register(dev);
	if (ret)
		goto fail;

	rtdm_lock_get_irqsave(&master->lock, c);
	list_add_tail(&slave->next, &master->slaves);
	rtdm_lock_put_irqrestore(&master->lock, c);

	return 0;
fail:
	kfree(dev->label);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_i2c_add_remote_slave);

void rtdm_i2c_remove_remote_slave(struct rtdm_i2c_remote_slave *slave)
{
	struct rtdm_i2c_master *master = slave->master;
	struct rtdm_device *dev;
	rtdm_lockctx_t c;
	
	mutex_destroy(&slave->ctl_lock);
	rtdm_lock_get_irqsave(&master->lock, c);
	list_del(&slave->next);
	rtdm_lock_put_irqrestore(&master->lock, c);
	dev = &slave->dev;
	rtdm_dev_unregister(dev);
	kfree(dev->label);
}
EXPORT_SYMBOL_GPL(rtdm_i2c_remove_remote_slave);

static int i2c_device_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct rtdm_i2c_remote_slave *slave;
	struct rtdm_i2c_master *master;
	int ret;

	master = i2c_get_adapdata(i2c->adapter);
	if (master->devclass == NULL) {
		ret = __rtdm_i2c_setup_driver(master);
		if (ret)
			return ret;
	}

	slave = master->ops->attach_slave(master, i2c);
	if (IS_ERR(slave))
		return PTR_ERR(slave);

	i2c_set_clientdata(i2c, slave);

	return 0;
}

static int i2c_device_remove(struct i2c_client *i2c)
{
	struct rtdm_i2c_remote_slave *slave = i2c_get_clientdata(i2c);

	slave->master->ops->detach_slave(slave);

	return 0;
}

static const struct of_device_id i2c_device_match[] = {
	{
		.compatible = "rtdm-i2cdev",
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, i2c_device_match);

static struct i2c_driver i2c_device_driver = {
	.driver = {
		.name =	"rtdm_i2c_device",
		.owner = THIS_MODULE,
		.of_match_table = i2c_device_match,
	},
	.probe	= i2c_device_probe,
	.remove	= i2c_device_remove,
};

module_i2c_driver(i2c_device_driver);
