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
#define THETA0 		-17.5*M_PI/180
#define THETA1 		-48.5*M_PI/180
#define THETA2 		-M_PI/2
#define THETA3 		-5*M_PI/6
#define THETA4 		5*M_PI/6
#define THETA5 		M_PI/2
#define THETA6 		48.5*M_PI/180
#define THETA7 		17.5*M_PI/180
#define THRESHOLD 	200


static const float COS_THETA []={cosf(THETA0) ,cosf(THETA1) ,cosf(THETA2) ,cosf(THETA3) ,cosf(THETA4) ,cosf(THETA5) ,cosf(THETA6) ,cosf(THETA7)};
static const float SIN_THETA []={sinf(THETA0) ,sinf(THETA1) ,sinf(THETA2) ,sinf(THETA3) ,sinf(THETA4) ,sinf(THETA5) ,sinf(THETA6) ,sinf(THETA7)};
//wheights assigned for the different sensors
static const float WHEIGHT []={THETA7+(THETA6-THETA7)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA4-THETA5+M_PI/6 ,
		THETA4-THETA5+M_PI/6 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA7+(THETA6-THETA7)/2};




//////////Public functions//////////

/**
 * @brief	gives the angle of incidence when collinding with a surface
 *
 * @return	the angle of incidence of the collision (in rad)
 */
float angle_colision (void){
	float sum_cos=0;
	float sum_sin=0;
	for (uint8_t i=0;i<PROXIMITY_NB_CHANNELS;i++){
			sum_cos+=get_calibrated_prox(i)*COS_THETA[i]*WHEIGHT[i];
			sum_sin+=get_calibrated_prox(i)*SIN_THETA[i]*WHEIGHT[i];
	}
	float theta=atan2f(sum_sin,sum_cos);
	return theta ;
	if (theta>0){
		return 2*theta-M_PI;
	}
	else{
		return M_PI+2*theta;
	}
}

/**
 * @brief	verifies if there was a colision
 *
 * @return	true if there is a colision false if there is not
 */
bool colision_detected (void){
	for (uint8_t i =0 ; i<PROXIMITY_NB_CHANNELS ;i++){
		if (get_calibrated_prox(i) >THRESHOLD){
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
		if (get_calibrated_prox(3)<3 &&get_calibrated_prox(4)<3)
			return true;
		else
			return false;
		break;
	case BACK_RIGHT:
			if (get_calibrated_prox(2)<3 &&get_calibrated_prox(3)<3)
				return true;
			else
				return false;
			break;
	case RIGHT:
			if (get_calibrated_prox(1)<3 &&get_calibrated_prox(2)<3)
				return true;
			else
				return false;
			break;
	case FRONT_RIGHT:
			if (get_calibrated_prox(0)<3 &&get_calibrated_prox(1)<3)
				return true;
			else
				return false;
			break;
	case FRONT:
			if (get_calibrated_prox(7)<3 &&get_calibrated_prox(0)<3)
				return true;
			else
				return false;
			break;
	case FRONT_LEFT:
			if (get_calibrated_prox(6)<3 &&get_calibrated_prox(7)<3)
				return true;
			else
				return false;
			break;
	case LEFT:
			if (get_calibrated_prox(5)<3 &&get_calibrated_prox(6)<3)
				return true;
			else
				return false;
			break;
	case BACK_LEFT:
			if (get_calibrated_prox(4)<3 &&get_calibrated_prox(5)<3)
				return true;
			else
				return false;
			break;
	default :
		return false ;
	}
}
