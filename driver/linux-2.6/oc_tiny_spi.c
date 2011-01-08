/*
 * Opencore tiny_spi driver
 *
 * Copyright (C) 2011 Thomas Chou <thomas@wytron.com.tw>
 *
 * Based on spi_s3c24xx.c, which is:
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/io.h>
#include <linux/gpio.h>

#define DRV_NAME "oc_tiny_spi"

#define TINY_SPI_RXDATA	0
#define TINY_SPI_TXDATA	4
#define TINY_SPI_STATUS	8
#define TINY_SPI_CONTROL	12
#define TINY_SPI_BAUD	16

#define TINY_SPI_STATUS_TXE 0x1
#define TINY_SPI_STATUS_TXR 0x2

struct tiny_spi {
	/* bitbang has to be first */
	struct spi_bitbang bitbang;
	struct completion done;

	void __iomem *base;
	int irq;
	uint freq;
	uint baudwidth;
	uint baud;
	uint speed_hz;

	struct spi_master *master;
	struct resource *ioarea;
	struct device *dev;
};

static inline struct tiny_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static uint tiny_spi_baud(struct spi_device *spi, uint hz)
{
	struct tiny_spi *hw = to_hw(spi);
	uint baud;
	baud = DIV_ROUND_UP(hw->freq, hz * 2) - 1;
	if (baud > (1 << hw->baudwidth) - 1)
		baud = (1 << hw->baudwidth) - 1;
	return baud;
}

static void tiny_spi_chipselect(struct spi_device *spi, int is_active)
{
	gpio_set_value(spi->chip_select, (spi->mode & SPI_CS_HIGH) ? is_active : !is_active);
}

static int tiny_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct tiny_spi *hw = to_hw(spi);
	uint baud = hw->baud;
	if (t) {
		dev_dbg(&spi->dev, "%s: %u bpw, %d hz\n",
			__func__, t->bits_per_word, t->speed_hz);
		if (t->speed_hz && t->speed_hz != hw->speed_hz)
			baud = tiny_spi_baud(spi, t->speed_hz);
	}
	writel(baud, hw->base + TINY_SPI_BAUD);
	return 0;
}

static int tiny_spi_setup(struct spi_device *spi)
{
	struct tiny_spi *hw = to_hw(spi);

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__func__, spi->mode, spi->bits_per_word, spi->max_speed_hz);
	if (spi->max_speed_hz != hw->speed_hz) {
		hw->speed_hz = spi->max_speed_hz;
		hw->baud = tiny_spi_baud(spi, hw->speed_hz);
	}
	return 0;
}

#ifndef CONFIG_TINY_SPI_IDLE_VAL
# define CONFIG_TINY_SPI_IDLE_VAL 0xff
#endif

static int tiny_spi_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct tiny_spi *hw = to_hw(spi);
	const u8 *txp = t->tx_buf;
	u8 *rxp = t->rx_buf;
	uint i;

	dev_dbg(&spi->dev, "%s: tx %p, rx %p, len %d\n",
		__func__, t->tx_buf, t->rx_buf, t->len);

	/* we need to tighten the transfer loop */
	if (txp && rxp) {
		writeb(*txp++, hw->base + TINY_SPI_TXDATA);
		if (t->len > 1) {
			writeb(*txp++, hw->base + TINY_SPI_TXDATA);
			for (i = 2; i < t->len; i++) {
				u8 rx, tx = *txp++;
				while (!(readb(hw->base + TINY_SPI_STATUS) &
					 TINY_SPI_STATUS_TXR))
					cpu_relax();
				rx = readb(hw->base + TINY_SPI_TXDATA);
				writeb(tx, hw->base + TINY_SPI_TXDATA);
				*rxp++ = rx;
			}
			while (!(readb(hw->base + TINY_SPI_STATUS) &
				 TINY_SPI_STATUS_TXR))
				cpu_relax();
			*rxp++ = readb(hw->base + TINY_SPI_TXDATA);
		}
		while (!(readb(hw->base + TINY_SPI_STATUS) &
			 TINY_SPI_STATUS_TXE))
			cpu_relax();
		*rxp++ = readb(hw->base + TINY_SPI_RXDATA);
	} else if (rxp) {
		writeb(CONFIG_TINY_SPI_IDLE_VAL, hw->base + TINY_SPI_TXDATA);
		if (t->len > 1) {
			writeb(CONFIG_TINY_SPI_IDLE_VAL,
			       hw->base + TINY_SPI_TXDATA);
			for (i = 2; i < t->len; i++) {
				u8 rx;
				while (!(readb(hw->base + TINY_SPI_STATUS) &
					 TINY_SPI_STATUS_TXR))
					cpu_relax();
				rx = readb(hw->base + TINY_SPI_TXDATA);
				writeb(CONFIG_TINY_SPI_IDLE_VAL,
				       hw->base + TINY_SPI_TXDATA);
				*rxp++ = rx;
			}
			while (!(readb(hw->base + TINY_SPI_STATUS) &
				 TINY_SPI_STATUS_TXR))
				cpu_relax();
			*rxp++ = readb(hw->base + TINY_SPI_TXDATA);
		}
		while (!(readb(hw->base + TINY_SPI_STATUS) &
			 TINY_SPI_STATUS_TXE))
			cpu_relax();
		*rxp++ = readb(hw->base + TINY_SPI_RXDATA);
	} else if (txp) {
		writeb(*txp++, hw->base + TINY_SPI_TXDATA);
		if (t->len > 1) {
			writeb(*txp++, hw->base + TINY_SPI_TXDATA);
			for (i = 2; i < t->len; i++) {
				u8 tx = *txp++;
				while (!(readb(hw->base + TINY_SPI_STATUS) &
					 TINY_SPI_STATUS_TXR))
					cpu_relax();
				writeb(tx, hw->base + TINY_SPI_TXDATA);
			}
		}
		while (!(readb(hw->base + TINY_SPI_STATUS) &
			 TINY_SPI_STATUS_TXE))
			cpu_relax();
	} else {
		writeb(CONFIG_TINY_SPI_IDLE_VAL, hw->base + TINY_SPI_TXDATA);
		if (t->len > 1) {
			writeb(CONFIG_TINY_SPI_IDLE_VAL,
			       hw->base + TINY_SPI_TXDATA);
			for (i = 2; i < t->len; i++) {
				while (!(readb(hw->base + TINY_SPI_STATUS) &
					 TINY_SPI_STATUS_TXR))
					cpu_relax();
				writeb(CONFIG_TINY_SPI_IDLE_VAL,
				       hw->base + TINY_SPI_TXDATA);
			}
		}
		while (!(readb(hw->base + TINY_SPI_STATUS) &
			 TINY_SPI_STATUS_TXE))
			cpu_relax();
	}
	return t->len;
}

static irqreturn_t tiny_spi_irq(int irq, void *dev)
{
	struct tiny_spi *hw = dev;
	writeb(0, hw->base + TINY_SPI_STATUS);
	complete(&hw->done);
	return IRQ_HANDLED;
}

static int __init tiny_spi_probe(struct platform_device *pdev)
{
	struct tiny_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct tiny_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct tiny_spi));

	hw->master = spi_master_get(master);
	hw->dev = &pdev->dev;

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the master state. */
	master->bus_num = pdev->id;
	master->num_chipselect = 256;
	master->mode_bits = SPI_CS_HIGH;

	/* setup the state for the bitbang driver */

	hw->bitbang.master = hw->master;
	hw->bitbang.setup_transfer = tiny_spi_setup_transfer;
	hw->bitbang.chipselect = tiny_spi_chipselect;
	hw->bitbang.txrx_bufs = tiny_spi_txrx_bufs;
	hw->bitbang.master->setup = tiny_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start) + 1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->base =
	    ioremap(res->start, (res->end - res->start) + 1);
	if (hw->base == 0) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	err = request_irq(hw->irq, tiny_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	hw->freq = 100000000;
	hw->baudwidth = 7;
	hw->baud = 1;

	dev_info(hw->dev, "base %p, irq %d\n", hw->base, hw->irq);

	/* register our spi controller */
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:
	free_irq(hw->irq, hw);
err_no_irq:
	iounmap((void *)hw->base);
err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);
err_no_iores:
	spi_master_put(hw->master);;
err_nomem:
	return err;
}

static int __exit tiny_spi_remove(struct platform_device *dev)
{
	struct tiny_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	free_irq(hw->irq, hw);
	iounmap((void *)hw->base);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

static struct platform_driver tiny_spidrv = {
	.remove = __exit_p(tiny_spi_remove),
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = NULL,
	},
};

static int __init tiny_spi_init(void)
{
	return platform_driver_probe(&tiny_spidrv, tiny_spi_probe);
}

static void __exit tiny_spi_exit(void)
{
	platform_driver_unregister(&tiny_spidrv);
}

module_init(tiny_spi_init);
module_exit(tiny_spi_exit);

MODULE_DESCRIPTION("Opencore tiny_spi driver");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
