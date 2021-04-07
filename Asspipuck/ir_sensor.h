/*
 * ir_sensor.h
 *
 *  Created on: 7 avr. 2021
 *      Author: sami bouziri
 */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <stdbool.h>

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

bool colision_detected (void);



#endif /* IR_SENSOR_H_ */
