# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc


obj-$(CONFIG_BBK_FP_ID)		+= fp_id.o
obj-$(CONFIG_BBK_FP_ID)		+= vfp_spi.o

ifdef CONFIG_BBK_FPC1511
obj-y					+= fpc1511/
endif

ifdef CONFIG_BBK_GF3208
obj-y	+= gf3208/
endif

ifdef CONFIG_BBK_ICNF7312
obj-y	+= icnf7312/
endif

ifdef CONFIG_BBK_FT9391
obj-y	+= ft9391/
endif

ifdef CONFIG_BBK_GF3626
obj-y	+= gf3626/
endif

