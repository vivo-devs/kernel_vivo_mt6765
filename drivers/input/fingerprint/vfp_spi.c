#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/io.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

/* MTK header */
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif


/**************************defination******************************/
#define VFP_DEV_NAME        "mtfp_spi"


#ifdef CONFIG_SPI_MT65XX
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

struct vfp_device {
	struct spi_device *spi;
#ifndef CONFIG_SPI_MT65XX
	struct mt_chip_conf spi_mcc;
#endif
};

static struct vfp_device g_vdev;

/*************************************************************/

#ifdef CONFIG_OF
static const struct of_device_id vfp_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{},
};
MODULE_DEVICE_TABLE(of, vfp_of_match);
#endif

#ifndef CONFIG_SPI_MT65XX
static const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 50, /* 1MHz */
	.low_time = 50,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,
	/* .com_mod = DMA_TRANSFER, */
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

void vfp_spi_clk_enable(uint8_t bonoff)
{
#ifdef CONFIG_MTK_CLKMGR
	if (bonoff) {
		enable_clock(MT_CG_PERI_SPI0, "spi");
		} else {
		disable_clock(MT_CG_PERI_SPI0, "spi");
	}
#else
	static int count;
	struct vfp_device *vfp_dev = &g_vdev;

	if (NULL == vfp_dev->spi) {
		pr_err("%s, spi is null, enable or disable clk failed.");
		return;
	}
	if (bonoff && (count == 0)) {
		mt_spi_enable_master_clk(vfp_dev->spi);
		count = 1;
		pr_info("clock enable");
	} else if ((count > 0) && (bonoff == 0)) {
		mt_spi_disable_master_clk(vfp_dev->spi);
		count = 0;
		pr_info("clock disable");
	}
#endif
}
EXPORT_SYMBOL(vfp_spi_clk_enable);

static int vfp_probe(struct spi_device *spi)
{
	struct vfp_device *vfp_dev =  &g_vdev;

	vfp_dev->spi = spi;

	/* setup SPI parameters */
	/* CPOL=CPHA=0, speed 1MHz */
	vfp_dev->spi->mode = SPI_MODE_0;
	vfp_dev->spi->bits_per_word = 8;
	vfp_dev->spi->max_speed_hz = 1 * 1000 * 1000;
#ifndef CONFIG_SPI_MT65XX
	memcpy(&vfp_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
	vfp_dev->spi->controller_data = (void *)&vfp_dev->spi_mcc;
	spi_setup(vfp_dev->spi);
#endif
	spi_set_drvdata(spi, vfp_dev);

	vfp_spi_clk_enable(0);
	pr_info("%s okay.", __func__);
	return 0;
}

static int vfp_remove(struct spi_device *spi)
{
	struct vfp_device *vfp_dev = NULL;

	pr_info("%s enter.", __func__);
	vfp_dev = spi_get_drvdata(spi);
	vfp_spi_clk_enable(0);

	spi_set_drvdata(spi, NULL);
	vfp_dev->spi = NULL;

	kfree(vfp_dev);
	return 0;
}

/*-------------------------------------------------------------------------*/
static struct spi_driver vfp_spi_driver = {
	.driver = {
		.name = VFP_DEV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = vfp_of_match,
#endif
	},
	.probe = vfp_probe,
	.remove = vfp_remove,
};

static int __init vfp_init(void)
{
	int status = 0;
	//pr_info("%s enter.", __func__);

	struct device_node *node = of_find_compatible_node(NULL, NULL, "fp-id");
	if (node) {
		if (of_property_read_bool(node, "vivo,support_soft_fingerprint_id")) {
			status = spi_register_driver(&vfp_spi_driver);
			if (status < 0) {
				pr_err("%s, Failed to register SPI driver.", __func__);
				return -EINVAL;
			}
			pr_info("%s, support software detect fpid, register SPI driver success.", __func__);
		} else {
			status = -EINVAL;
			pr_err("%s, not support software detect fpid.", __func__);
		}
	} else {
		status = -EINVAL;
		pr_err("%s, get node[fp_id] fail.", __func__);
	}

	return status;
}

// module_init(vfp_init);
late_initcall(vfp_init);

static void __exit vfp_exit(void)
{
	spi_unregister_driver(&vfp_spi_driver);
}
module_exit(vfp_exit);


MODULE_AUTHOR("vivo fingerprint team");
MODULE_DESCRIPTION("vivo mtk spi driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:vivo mtkfp_spi");
