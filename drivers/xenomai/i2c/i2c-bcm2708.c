/**
 * I/O handling lifted from drivers/i2c/i2c-bcm2709.c:
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2015 Martin Sperl
 *
 * RTDM integration by:
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include "i2c-master.h"

static bool combined = false;
module_param(combined, bool, 0644);
MODULE_PARM_DESC(combined, "Use combined transactions");

/* BSC register offsets */
#define BSC_C			0x00
#define BSC_S			0x04
#define BSC_DLEN		0x08
#define BSC_A			0x0c
#define BSC_FIFO		0x10
#define BSC_DIV			0x14
#define BSC_DEL			0x18
#define BSC_CLKT		0x1c

/* Bitfields in BSC_C */
#define BSC_C_I2CEN		0x00008000
#define BSC_C_INTR		0x00000400
#define BSC_C_INTT		0x00000200
#define BSC_C_INTD		0x00000100
#define BSC_C_ST		0x00000080
#define BSC_C_CLEAR_1		0x00000020
#define BSC_C_CLEAR_2		0x00000010
#define BSC_C_READ		0x00000001

/* Bitfields in BSC_S */
#define BSC_S_CLKT		0x00000200
#define BSC_S_ERR		0x00000100
#define BSC_S_RXF		0x00000080
#define BSC_S_TXE		0x00000040
#define BSC_S_RXD		0x00000020
#define BSC_S_TXD		0x00000010
#define BSC_S_RXR		0x00000008
#define BSC_S_TXW		0x00000004
#define BSC_S_DONE		0x00000002
#define BSC_S_TA		0x00000001

#define I2C_WAIT_LOOP_COUNT	200

#define RTDM_SUBCLASS_BCM2708  1

struct i2c_master_bcm2708 {
	struct rtdm_i2c_master master;
	struct i2c_msg *msg;
    int nmsgs;
	int pos;
	bool error;
	void __iomem *regs;
	struct clk *clk;
	unsigned long clk_hz;
	rtdm_irq_t irqh;
    rtdm_lock_t lock;

	u32 cdiv;
	u32 clk_tout;
	rtdm_event_t transfer_done;
};

static inline struct i2c_master_bcm2708 *
to_master_bcm2708(struct rtdm_i2c_master *master)
{
	return container_of(master, struct i2c_master_bcm2708, master);
}

static inline struct device *
master_to_kdev(struct rtdm_i2c_master *master)
{
	return &master->kadapter->dev;
}

static inline u32 bcm2708_rd(struct i2c_master_bcm2708 *i2cm, unsigned reg)
{
	return readl(i2cm->regs + reg);
}

static inline void bcm2708_wr(struct i2c_master_bcm2708 *i2cm, unsigned reg, u32 val)
{
	writel(val, i2cm->regs + reg);
}

static inline void bcm2708_bsc_reset(struct i2c_master_bcm2708 *i2cm)
{
	bcm2708_wr(i2cm, BSC_C, 0);
	bcm2708_wr(i2cm, BSC_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
}

static inline void bcm2708_bsc_fifo_drain(struct i2c_master_bcm2708 *i2cm)
{
	while ((bcm2708_rd(i2cm, BSC_S) & BSC_S_RXD) && (i2cm->pos < i2cm->msg->len))
		i2cm->msg->buf[i2cm->pos++] = bcm2708_rd(i2cm, BSC_FIFO);
}

static inline void bcm2708_bsc_fifo_fill(struct i2c_master_bcm2708 *i2cm)
{
	while ((bcm2708_rd(i2cm, BSC_S) & BSC_S_TXD) && (i2cm->pos < i2cm->msg->len))
		bcm2708_wr(i2cm, BSC_FIFO, i2cm->msg->buf[i2cm->pos++]);
}

static inline int bcm2708_bsc_setup(struct i2c_master_bcm2708 *i2cm)
{
	u32 cdiv, s, clk_tout;
	u32 c = BSC_C_I2CEN | BSC_C_INTD | BSC_C_ST | BSC_C_CLEAR_1;
	int wait_loops = I2C_WAIT_LOOP_COUNT;

	/* Can't call clk_get_rate as it locks a mutex and here we are spinlocked.
	 * Use the value that we cached in the probe.
	 */
	cdiv = i2cm->cdiv;
	clk_tout = i2cm->clk_tout;

	if (i2cm->msg->flags & I2C_M_RD)
		c |= BSC_C_INTR | BSC_C_READ;
	else
		c |= BSC_C_INTT;

	bcm2708_wr(i2cm, BSC_CLKT, clk_tout);
	bcm2708_wr(i2cm, BSC_DIV, cdiv);
	bcm2708_wr(i2cm, BSC_A, i2cm->msg->addr);
	bcm2708_wr(i2cm, BSC_DLEN, i2cm->msg->len);
	if (combined)
	{
		/* Do the next two messages meet combined transaction criteria?
		   - Current message is a write, next message is a read
		   - Both messages to same slave address
		   - Write message can fit inside FIFO (16 bytes or less) */
		if ( (i2cm->nmsgs > 1) &&
			!(i2cm->msg[0].flags & I2C_M_RD) && (i2cm->msg[1].flags & I2C_M_RD) &&
			 (i2cm->msg[0].addr == i2cm->msg[1].addr) && (i2cm->msg[0].len <= 16)) {
			/* Fill FIFO with entire write message (16 byte FIFO) */
			while (i2cm->pos < i2cm->msg->len) {
				bcm2708_wr(i2cm, BSC_FIFO, i2cm->msg->buf[i2cm->pos++]);
			}
			/* Start write transfer (no interrupts, don't clear FIFO) */
			bcm2708_wr(i2cm, BSC_C, BSC_C_I2CEN | BSC_C_ST);

			/* poll for transfer start bit (should only take 1-20 polls) */
			do {
				s = bcm2708_rd(i2cm, BSC_S);
			} while (!(s & (BSC_S_TA | BSC_S_ERR | BSC_S_CLKT | BSC_S_DONE)) && --wait_loops >= 0);

			/* did we time out or some error occured? */
			if (wait_loops < 0 || (s & (BSC_S_ERR | BSC_S_CLKT))) {
				return -1;
			}

			/* Send next read message before the write transfer finishes. */
			i2cm->nmsgs--;
			i2cm->msg++;
			i2cm->pos = 0;
			bcm2708_wr(i2cm, BSC_DLEN, i2cm->msg->len);
			c = BSC_C_I2CEN | BSC_C_INTD | BSC_C_INTR | BSC_C_ST | BSC_C_READ;
		}
	}
	bcm2708_wr(i2cm, BSC_C, c);

	return 0;
}

static int bcm2708_i2c_interrupt(rtdm_irq_t *irqh)
{
	struct i2c_master_bcm2708 *i2cm;
    u32 s;
    int ret;
    bool handled = true;

	i2cm = rtdm_irq_get_arg(irqh, struct i2c_master_bcm2708);

	rtdm_lock_get(&i2cm->lock);

	/* we may see camera interrupts on the "other" I2C channel
		   Just return if we've not sent anything */
	if (!i2cm->nmsgs || !i2cm->msg) {
		goto early_exit;
	}

	s = bcm2708_rd(i2cm, BSC_S);

	if (s & (BSC_S_CLKT | BSC_S_ERR)) {
		bcm2708_bsc_reset(i2cm);
		i2cm->error = true;

		i2cm->msg = 0; /* to inform the that all work is done */
		i2cm->nmsgs = 0;
		/* wake up our bh */
		rtdm_event_signal(&i2cm->transfer_done);
	} else if (s & BSC_S_DONE) {
		i2cm->nmsgs--;

		if (i2cm->msg->flags & I2C_M_RD) {
			bcm2708_bsc_fifo_drain(i2cm);
		}

		bcm2708_bsc_reset(i2cm);

		if (i2cm->nmsgs) {
			/* advance to next message */
			i2cm->msg++;
			i2cm->pos = 0;
			ret = bcm2708_bsc_setup(i2cm);
			if (ret < 0) {
				bcm2708_bsc_reset(i2cm);
				i2cm->error = true;
				i2cm->msg = 0; /* to inform the that all work is done */
				i2cm->nmsgs = 0;
				/* wake up our bh */
                rtdm_event_signal(&i2cm->transfer_done);
				goto early_exit;
			}
		} else {
			i2cm->msg = 0; /* to inform the that all work is done */
			i2cm->nmsgs = 0;
			/* wake up our bh */
            rtdm_event_signal(&i2cm->transfer_done);
		}
	} else if (s & BSC_S_TXW) {
		bcm2708_bsc_fifo_fill(i2cm);
	} else if (s & BSC_S_RXR) {
		bcm2708_bsc_fifo_drain(i2cm);
	} else {
		handled = false;
	}

early_exit:
	rtdm_lock_put(&i2cm->lock);

	return handled ? RTDM_IRQ_HANDLED: RTDM_IRQ_NONE;
}

static int bcm2708_configure(struct rtdm_i2c_master *master)
{
    /*
	struct i2c_master_bcm2708 *i2cm = to_master_bcm2708(master);
	struct rtdm_i2c_config *config = &master->config;
    */

    //printk("do nothing here\n");

	return 0;
}

static int do_transfer_irq(struct rtdm_i2c_master *master)
{
	struct i2c_master_bcm2708 *i2cm = to_master_bcm2708(master);
	int ret;

	ret = bcm2708_bsc_setup(i2cm);

	/* check the result of the setup */
	if (ret < 0) {
		rtdm_printk("transfer setup timed out\n");
        return -1;
	}

	ret = rtdm_event_wait(&i2cm->transfer_done);
	if (ret) {
        bcm2708_bsc_reset(i2cm);
		return ret;
	}

    if (i2cm->error) {
        printk("i2c error\n");
        return -EIO;
    }

	return 0;
}

static int bcm2708_transfer(struct rtdm_i2c_master *master, struct i2c_msg *msgs, int num)
{
    struct i2c_master_bcm2708 *i2cm = to_master_bcm2708(master);
    rtdm_lockctx_t lock_ctx;
    
    rtdm_lock_get_irqsave(&i2cm->lock, lock_ctx);

	i2cm->msg = msgs;
	i2cm->pos = 0;
	i2cm->nmsgs = num;
	i2cm->error = false;
    
    rtdm_lock_put_irqrestore(&i2cm->lock, lock_ctx);

	return do_transfer_irq(master);
}

static ssize_t bcm2708_read(struct rtdm_i2c_master *master,
			    void *rx, size_t len)
{
	struct i2c_master_bcm2708 *i2cm = to_master_bcm2708(master);
    rtdm_lockctx_t lock_ctx;
    struct i2c_msg msgs = {
        .addr = master->config.addr,
        .flags = I2C_M_RD,
        .len = len,
        .buf = rx
    };

    rtdm_lock_get_irqsave(&i2cm->lock, lock_ctx);
    
    i2cm->msg = &msgs;
	i2cm->pos = 0;
	i2cm->nmsgs = 1;
	i2cm->error = false;
   
    rtdm_lock_put_irqrestore(&i2cm->lock, lock_ctx);

	return do_transfer_irq(master) ?: len;
}

static ssize_t bcm2708_write(struct rtdm_i2c_master *master,
			     const void *tx, size_t len)
{
	struct i2c_master_bcm2708 *i2cm = to_master_bcm2708(master);
    rtdm_lockctx_t lock_ctx;
    struct i2c_msg msgs = {
        .addr = master->config.addr,
        .flags = 0,
        .len = len,
        .buf = (void *)tx
    };

    rtdm_lock_get_irqsave(&i2cm->lock, lock_ctx);

    i2cm->msg = &msgs;
	i2cm->pos = 0;
	i2cm->nmsgs = 1;
	i2cm->error = false;

    rtdm_lock_put_irqrestore(&i2cm->lock, lock_ctx);

	return do_transfer_irq(master) ?: len;
}

static struct rtdm_i2c_master_ops bcm2708_master_ops = {
	.configure = bcm2708_configure,
	.transfer = bcm2708_transfer,
	.write = bcm2708_write,
	.read = bcm2708_read,
};

static int bcm2708_i2c_probe(struct platform_device *pdev)
{
	struct i2c_master_bcm2708 *i2cm;
	struct rtdm_i2c_master *master;
	struct i2c_adapter *kadapter;
	struct resource *r;
	int ret, irq;
	u32 baud, cdiv, clk_tout;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);
    
    baud = 1000;
    if (pdev->dev.of_node) {
        u32 bus_clk_rate;
        pdev->id = of_alias_get_id(pdev->dev.of_node, "i2c");
        if (pdev->id < 0) {
            dev_err(&pdev->dev, "alias is missing\n");
            return -EINVAL;
        }
        if (!of_property_read_u32(pdev->dev.of_node,
                    "clock-frequency", &bus_clk_rate))
            baud = bus_clk_rate;
        else
            dev_warn(&pdev->dev,
                    "Could not read clock-frequency property\n");
    }

    master = rtdm_i2c_alloc_master(&pdev->dev,
		   struct i2c_master_bcm2708, master);
	if (master == NULL)
		return -ENOMEM;

	master->subclass = RTDM_SUBCLASS_BCM2708;
	master->ops = &bcm2708_master_ops;
	platform_set_drvdata(pdev, master);

	kadapter = master->kadapter;
	kadapter->class = I2C_CLASS_DDC;
	kadapter->algo_data = NULL;
	kadapter->dev.parent = &pdev->dev;
	kadapter->nr = pdev->id;
	strlcpy(kadapter->name, dev_name(&pdev->dev), sizeof(kadapter->name));
	kadapter->dev.of_node = pdev->dev.of_node;
	dev_set_name(&kadapter->dev, "i2c%u", kadapter->nr);

	i2cm = container_of(master, struct i2c_master_bcm2708, master);
	rtdm_event_init(&i2cm->transfer_done, 0);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2cm->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(i2cm->regs)) {
		dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
		ret = PTR_ERR(i2cm->regs);
		goto fail;
	}
	
	i2cm->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2cm->clk)) {
		ret = PTR_ERR(i2cm->clk);
		goto fail;
	}

	i2cm->clk_hz = clk_get_rate(i2cm->clk);
    cdiv = i2cm->clk_hz / baud;
	if (cdiv > 0xffff) {
		cdiv = 0xffff;
		baud = i2cm->clk_hz / cdiv;
	}

	clk_tout = 35/1000*baud; //35ms timeout as per SMBus specs.
	if (clk_tout > 0xffff)
		clk_tout = 0xffff;

	i2cm->cdiv = cdiv;
	i2cm->clk_tout = clk_tout;

    irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0) {
		ret = irq ?: -ENODEV;
		goto fail;
	}

	clk_prepare_enable(i2cm->clk);

	ret = rtdm_irq_request(&i2cm->irqh, irq,
			       bcm2708_i2c_interrupt, XN_IRQTYPE_SHARED,
			       dev_name(&pdev->dev), i2cm);
	if (ret) {
		dev_err(&pdev->dev, "%s: cannot request IRQ%d\n",
			__func__, irq);
		goto fail_unclk;
	}

	bcm2708_bsc_reset(i2cm);

	ret = rtdm_i2c_add_master(&i2cm->master);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to add master\n",
			__func__);
		goto fail_unclk;
	}

	return 0;

fail_unclk:
	clk_disable_unprepare(i2cm->clk);
fail:
    //todo
	//i2c_master_put(kadapter);

	return ret;
}

static int bcm2708_i2c_remove(struct platform_device *pdev)
{
	struct rtdm_i2c_master *master = platform_get_drvdata(pdev);
	struct i2c_master_bcm2708 *i2cm;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	platform_set_drvdata(pdev, NULL);

	i2cm = container_of(master, struct i2c_master_bcm2708, master);

	rtdm_irq_free(&i2cm->irqh);

	clk_disable_unprepare(i2cm->clk);

    clk_put(i2cm->clk);

	rtdm_i2c_remove_master(master);

	return 0;
}

static const struct of_device_id bcm2708_i2c_match[] = {
	{
		.compatible = "brcm,bcm2708-i2c",
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcm2708_i2c_match);

static struct platform_driver bcm2708_i2c_driver = {
	.driver		= {
		.name		= "i2c-bcm2708",
		.of_match_table	= bcm2708_i2c_match,
	},
	.probe		= bcm2708_i2c_probe,
	.remove		= bcm2708_i2c_remove,
};
module_platform_driver(bcm2708_i2c_driver);

MODULE_LICENSE("GPL");
