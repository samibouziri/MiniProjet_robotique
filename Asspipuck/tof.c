/*
 * tof.c
 *
 *  Created on: 4 mai 2021
 *      Author: sami bouziri
 */

#include "ch.h"
#include "hal.h"
#include "tof.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/VL53L0X/Api/core/inc/vl53l0x_api.h"
#include "shell.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"

static uint16_t dist_mm = 0;
static thread_t *distThd;
static bool VL53L0X_configured = false;

//////////////////// PUBLIC FUNCTIONS /////////////////////////
static THD_WORKING_AREA(waTOFThd, 512);
static THD_FUNCTION(TOFThd, arg) {

	chRegSetThreadName("TOF Thd");
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	(void)arg;
	static VL53L0X_Dev_t device;

	device.I2cDevAddr = VL53L0X_ADDR;

	status = VL53L0X_init(&device);

	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configAccuracy(&device, VL53L0X_HIGH_ACCURACY);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configured = true;
	}

    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false) {
    	if(VL53L0X_configured){
    		VL53L0X_getLastMeasure(&device);
   			dist_mm = device.Data.LastRangeMeasure.RangeMilliMeter;
    	}
		chThdSleepMilliseconds(200);
    }
}

void TOF_start(void){

	if(VL53L0X_configured) {
		return;
	}

	i2c_start();

	distThd = chThdCreateStatic(waTOFThd,
                     sizeof(waTOFThd),
                     NORMALPRIO + 10,
                     TOFThd,
                     NULL);
}

void TOF_stop(void) {
    chThdTerminate(distThd);
    chThdWait(distThd);
    distThd = NULL;
    VL53L0X_configured = false;
}

uint16_t TOF_get_dist_mm(void) {
	return dist_mm;
}

