#
# Makefile for the fingerprint device.
#
# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
subdir-ccflags-y += -I$(srctree)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
#include $(srctree)/drivers/misc/mediatek/gud/Makefile.include

obj-$(CONFIG_BBK_GF3208)	+= gf_spi_tee.o
