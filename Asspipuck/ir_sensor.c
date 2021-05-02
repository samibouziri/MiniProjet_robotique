/*
 * ir_sensor.c
 *
 *  Created on: 7 avr. 2021
 *      Author: sami bouziri
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
static const float WHEIGHT []={THETA7+(THETA6-THETA7)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA4-THETA5+M_PI/6 ,
		THETA4-THETA5+M_PI/6 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA7+(THETA6-THETA7)/2};

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

/**
 * @brief	detects the region of collision depending on the angle
 * 			of collision
 * @param 	angle (in rad): angle of collision
 * @return 	region of collision
 */
region get_region(float angle){
	if (angle>-M_PI && angle<=THETA3)
		return BACK;

	if (angle>THETA3 && angle<=THETA2)
		return BACK_RIGHT;

	if (angle>THETA2 && angle<=THETA1)
		return RIGHT;

	if (angle>THETA1 && angle<=THETA0)
		return FRONT_RIGHT;

	if (angle>THETA0 && angle<=THETA7)
		return FRONT;

	if (angle>THETA7 && angle<=THETA6)
		return FRONT_LEFT;

	if (angle>THETA6 && angle<=THETA5)
		return LEFT;

	if (angle>THETA5 && angle<=THETA4)
		return BACK_LEFT;

	return BACK;

}


/**
 * @brief	indicates if the path is free or not based on the region
 * 			that has the highest value of detection
 * @param 	angle (in rad): angle of collision
 * @return 	true if the path is free, false otherwise
 */
bool is_path_free(float angle){
	switch (get_region(angle)){
	case BACK:
		if (get_calibrated_prox(SENSOR_4)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_5)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case BACK_RIGHT:
		if (get_calibrated_prox(SENSOR_3)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_4)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case RIGHT:
		if (get_calibrated_prox(SENSOR_2)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_3)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case FRONT_RIGHT:
		if (get_calibrated_prox(SENSOR_1)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_2)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case FRONT:
		if (get_calibrated_prox(SENSOR_8)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_1)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case FRONT_LEFT:
		if (get_calibrated_prox(SENSOR_7)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_8)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case LEFT:
		if (get_calibrated_prox(SENSOR_6)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_7)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	case BACK_LEFT:
		if (get_calibrated_prox(SENSOR_5)<NO_OBS_VAL &&get_calibrated_prox(SENSOR_6)<NO_OBS_VAL)
			return true;
		else
			return false;
		break;
	default :
		return false ;
	}
}
