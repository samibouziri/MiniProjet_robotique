/*	Project: AspiPuck
 * 	ir_sensor.c
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was created by the authors.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <sensors/proximity.h>
#include <ir_sensor.h>

//angle of positionning of the different sensors
#define THETA0 		-15*M_PI/180
#define THETA1 		-45*M_PI/180
#define THETA2 		-M_PI/2.f
#define THETA3 		-5*M_PI/6.f
#define THETA4 		5*M_PI/6.f
#define THETA5 		M_PI/2.f
#define THETA6 		45*M_PI/180
#define THETA7 		15*M_PI/180
#define NO_OBS_VAL	3

//wheights assigned for the different sensors
static const float WHEIGHT []={1,1,2,2,2,2,1,1};

/**
 * @brief	return the angle of a sensor
 *@param 	idx the index the of the targeted sensor
 * @return	the angle of the sensor number idx
 */

float get_theta(uint8_t idx){
	float theta1=0;
	switch (idx){
	case SENSOR_1:
		theta1=THETA0;
		break;
	case SENSOR_2:
		theta1=THETA1;
		break;
	case SENSOR_3:
		theta1=THETA2;
		break;
	case SENSOR_4:
		theta1=THETA3;
		break;
	case SENSOR_5:
		theta1=THETA4;
		break;
	case SENSOR_6:
		theta1=THETA5;
		break;
	case SENSOR_7:
		theta1=THETA6;
		break;
	case SENSOR_8:
		theta1=THETA7;
		break;
	}
	return theta1;
}

//////////Public functions//////////

/**
 * @brief	gives the angle of incidence when collinding with a surface
 *
 * @return	the angle of incidence of the collision (in rad)
 */

float angle_colision (void){
	float wheighted_sum =0;
	float sum=0;
	for (uint8_t i=0;i<PROXIMITY_NB_CHANNELS;i++){
		wheighted_sum+=get_calibrated_prox(i)*get_theta(i)*WHEIGHT[i];
		sum+=get_calibrated_prox(i)*WHEIGHT[i];
	}
	return wheighted_sum/sum;
}

/**
 * @brief	verifies if there was a colision
 *
 * @return	true if there is a colision false if there is not
 */

bool colision_detected (int threshold){
	for (uint8_t i =0 ; i<PROXIMITY_NB_CHANNELS ;i++){
		if (get_calibrated_prox(i) >threshold){
			return true;
		}
	}
	return false;
}



/**
 * @brief	detect if a sensor is in close proximity
 * 			of an obstacle with a certain threshold
 *
 * @param 	sensor to check the proximity of
 * @param 	threshold to return true
 * @return	true if in proximity and false otherwise
 */
bool sensor_close_obstacle (sensors_t sensor, uint16_t threshold)
{
	if (get_calibrated_prox(sensor) >= threshold)
		return true;
	else return false;

}
