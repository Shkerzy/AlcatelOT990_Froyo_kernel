/*
 * Definitions for akm8976 compass chip.
 */
#ifndef BMA150_H
#define BMA150_H

#include <linux/ioctl.h>


#define BMA150IO				0xB1
#define RBUFF_SIZES				31  /* Rx buffer size */

/* IOCTLs for AKM library */
#define BMA_IOCTL_GETDATA               _IOR(BMA150IO, 0x08, char[RBUFF_SIZES+1])
#define BMA_IOCTL_GET_DELAY             _IOR(BMA150IO, 0x30, short)
/* IOCTLs for APPs */
#define BMA_IOCTL_APP_SET_DELAY		_IOW(BMA150IO, 0x18, short)
#define BMA_IOCTL_APP_GET_DELAY		BMA_IOCTL_GET_DELAY

/* IOCTLs for pedometer */


/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct bma150_accel_data {
	int              accel_x;
	int              accel_y;
	int              accel_z;
	int              temp;
};

/** bma150 acceleration data 
 *  \brief Structure containing acceleration values for x,y and z-axis in signed short
 **
 */

typedef struct  {
				short x, /**< holds x-axis acceleration data sign extended. Range -512 to 511. */
					  y, /**< holds y-axis acceleration data sign extended. Range -512 to 511. */
					  z; /**< holds z-axis acceleration data sign extended. Range -512 to 511. */
} bma150acc_t;
#endif

