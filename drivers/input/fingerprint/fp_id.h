#ifndef __FP_ID_H
#define __FP_ID_H
/*for kernel
denominate method: ##VENDOR_##MODULE*/
#define FPC_BASE				0x00000100
#define FPC_FPC1022				(FPC_BASE + 1)
#define FPC_FPC1245				(FPC_BASE + 2)
#define FPC_FPC1229				(FPC_BASE + 3)
#define FPC_FPC1511				(FPC_BASE + 4)
#define FPC_FPC1540				(FPC_BASE + 5)

#define GOODIX_BASE				0x00000200
#define GOODIX_GF318M			(GOODIX_BASE + 1)
#define GOODIX_GF3208			(GOODIX_BASE + 2)
#define GOODIX_GF5126M			(GOODIX_BASE + 3)
#define GOODIX_GF5216C			(GOODIX_BASE + 4)
#define GOODIX_GF5269			(GOODIX_BASE + 5)
#define GOODIX_GF5288			(GOODIX_BASE + 6)
#define GOODIX_GF3658			(GOODIX_BASE + 7)
#define GOODIX_GF3626			(GOODIX_BASE + 8)

#define SILEAD_BASE				0x00000400
#define SILEAD_GSL6165			(SILEAD_BASE + 1)

#define FOCALTECH_BASE			0x00000500
#define FOCALTECH_FT9391		(FOCALTECH_BASE + 1)

#define CHIPONE_BASE			0x00000600
#define CHIPONE_ICNF7312		(CHIPONE_BASE + 1)

/*for frameworks
denominate method: ##VENDOR_FRAME_ID*/
#define FPC_FRAME_ID			1
#define GOODIX_FRAME_ID			2
#define SILEAD_FRAME_ID			4
#define FOCALTECH_FRAME_ID		5
#define CHIPONE_FRAME_ID		6

#define     FP_AVDD_EN_1V8        (GPIO54 | 0x80000000)


int get_fp_id(void);

#endif

