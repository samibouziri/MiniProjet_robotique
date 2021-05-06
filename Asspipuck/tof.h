/*
 * tof.h
 *
 *  Created on: 4 mai 2021
 *      Author: sami bouziri
 */

#ifndef TOF_H_
#define TOF_H_

#include "sensors/VL53L0X/Api/core/inc/vl53l0x_api.h"

#define USE_I2C_2V8

#define VL53L0X_ADDR 0x52

//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////
/**
 * @brief Init a thread which uses the distance sensor to
 * continuoulsy measure the distance.
 */
void TOF_start(void);

/**
* @brief   Stop the distance measurement.
*
*/
void TOF_stop(void);

/**
 * @brief 			Return the last distance measured in mm
 *
 * @return 			Last distance measured in mm
 */
uint16_t TOF_get_dist_mm(void);
#endif /* TOF_H_ */
