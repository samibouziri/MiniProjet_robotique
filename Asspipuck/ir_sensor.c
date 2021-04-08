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
#define THRESHOLD 	600
#define CLOSE_THR	100

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
 * @brief	returns detected value of the sensor
 *
 * @param	sensor that we want the value of
 */
float sensor_detection(sensors_t sensor)
{
return get_calibrated_prox(sensor);
}


/**
 * @brief	detect if a sensor is in close proximity
 * 			of an obstacle
 *
 * @param 	sensor to check the proximity of
 * @return	true if in proximity and false otherwise
 */
bool sensor_close_obstacle (sensors_t sensor)
{
	if (sensor_detection(sensor) <=CLOSE_THR)
		return true;
	else return false;

}





