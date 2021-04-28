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
#define THRESHOLD 	200


//static const float COS_THETA []={cosf(THETA0) ,cosf(THETA1) ,cosf(THETA2) ,cosf(THETA3) ,cosf(THETA4) ,cosf(THETA5) ,cosf(THETA6) ,cosf(THETA7)};
//static const float SIN_THETA []={sinf(THETA0) ,sinf(THETA1) ,sinf(THETA2) ,sinf(THETA3) ,sinf(THETA4) ,sinf(THETA5) ,sinf(THETA6) ,sinf(THETA7)};
//wheights assigned for the different sensors
static const float WHEIGHT []={THETA7+(THETA6-THETA7)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA4-THETA5+M_PI/6 ,
		THETA4-THETA5+M_PI/6 ,THETA5-THETA6+(THETA4-THETA5)/2 ,THETA6-THETA7+(THETA5-THETA6)/2 ,THETA7+(THETA6-THETA7)/2};


float get_theta(uint8_t idx){
	float theta1=0;
	switch (idx){
	case 0:
		theta1=THETA0;
		break;
	case 1:
		theta1=THETA1;
		break;
	case 2:
		theta1=THETA2;
		break;
	case 3:
		theta1=THETA3;
		break;
	case 4:
		theta1=THETA4;
		break;
	case 5:
		theta1=THETA5;
		break;
	case 6:
		theta1=THETA6;
		break;
	case 7:
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

/*float angle_colision (void){
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
}*/

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
