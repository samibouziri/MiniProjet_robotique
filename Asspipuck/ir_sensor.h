/*	Project: AspiPuck
 * 	ir_sensor.h
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was created by the authors.
 */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <stdbool.h>


typedef enum{
	SENSOR_1,
	SENSOR_2,
	SENSOR_3,
	SENSOR_4,
	SENSOR_5,
	SENSOR_6,
	SENSOR_7,
	SENSOR_8,
}sensors_t;

/**
 * @brief	gives the angle of incidence when collinding with a surface
 *
 * @return	the angle of incidence of the collision (in rad)
 */

float angle_colision (void);

/**
 * @brief	verifies if there was a colision
 *
 * @return	true if there is a colision false if there is not
 */

bool colision_detected (int threshold);


/**
 * @brief	detect if a sensor is in close proximity
 * 			of an obstacle with a certain threshold
 *
 * @param 	sensor to check the proximity of
 * @param 	threshold to return true
 * @return	true if in proximity and false otherwise
 */

bool sensor_close_obstacle (sensors_t sensor, uint16_t threshold);

#endif /* IR_SENSOR_H_ */
