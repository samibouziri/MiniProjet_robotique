/*
 * movements.c
 *
 *  Created on: 7 avr. 2021
 *      Author: sami bouziri
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <motors.h>
#include <movements.h>

#define NSTEP_ONE_TURN      1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     13.f
#define TURN_STEP			(PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN //nb steps for one full turn
#define MAX_ANGLE			360

static float SIN[MAX_ANGLE];
static float COS[MAX_ANGLE];

static float x=0;
static float y=0;
static int16_t angle=0;

//////////Private functions//////////

void init_sin (void){
	for (uint16_t i=0; i<MAX_ANGLE; i++){
		SIN[i]=sinf(i*M_PI/180);
		chprintf((BaseSequentialStream *)&SD3, "sin[%d]=%f",i,SIN[i]);
	}
}

void init_cos (void){
	for (uint16_t i=0; i<MAX_ANGLE; i++){
		COS[i]=cosf(i*M_PI/180);
		chprintf((BaseSequentialStream *)&SD3, "cos[%d]=%f",i,COS[i]);
	}
}

static THD_WORKING_AREA(waRobotPosition, 256);
static THD_FUNCTION(RobotPosition, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int32_t last_right_motor_pos=0;
	int32_t last_left_motor_pos=0;

	float amplitude =0;

	while(1){
		chThdSleepMilliseconds(10);
		amplitude = get_translation(last_right_motor_pos,last_left_motor_pos);
		angle+= get_rotation(last_right_motor_pos,last_left_motor_pos);
		last_right_motor_pos=right_motor_get_pos();
		last_left_motor_pos=left_motor_get_pos();
		angle %= MAX_ANGLE;
		if (angle<0){
			angle +=360;
		}

		x+=amplitude*COS[angle];
		y+=amplitude*SIN[angle];



	}
}

//////////Public functions/////////


/**
 * @brief rotates the e_puck with a certain angle and speed
 *
 * @param angle angle of rotation (in rad)
 * @param speed speed of rotation (in step/s)
 */
void rotate_rad(float angle, float speed)
{
	position_mode(angle*WHEEL_DISTANCE/2, -angle*WHEEL_DISTANCE/2, abs(speed),  abs(speed));

}

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
float angle_reflection (float angle_colision){
	if (angle_colision>M_PI/2)
		angle_colision=M_PI/2;
	if (angle_colision<-M_PI/2)
		angle_colision=-M_PI/2;
	if (angle_colision>0){
		return 2*angle_colision-M_PI;
	}
	else{
		return M_PI+2*angle_colision;
	}
}


void position_mode(float pos_r, float pos_l, float speed_r,  float speed_l)
{
	bool stop_r=false;
	bool stop_l=false;

	int32_t right_pos =right_motor_get_pos();
	int32_t left_pos =left_motor_get_pos();

	if (pos_r<0)
		speed_r=-abs(speed_r);
	else
		speed_r=abs(speed_r);
	if (pos_l<0)
		speed_l=-abs(speed_l);
	else
		speed_l=abs(speed_l);
	right_motor_set_speed(speed_r);
	left_motor_set_speed(speed_l);

	while (1)
	{
		if (abs(right_motor_get_pos()-right_pos)>=abs(pos_r*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
		{
			stop_r=true;
			right_motor_set_speed(0);
		}
		if (abs(left_motor_get_pos()-left_pos)>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
		{
			stop_l=true;
			left_motor_set_speed(0);
		}
		if (stop_r && stop_l) break;
	}

}


/**
 * @brief	returns the translation of the epuck from the initalization point
 *
 * @return	the translation in cm
 */
float get_translation (int32_t last_right_motor_pos,int32_t last_left_motor_pos){
	return step_to_cm(((left_motor_get_pos()-last_left_motor_pos)+(right_motor_get_pos()-last_right_motor_pos))/2);

}

/**
 * @brief	returns the angle of rotation of the epuck from the initalization point
 *
 * @return	the angle of rotation in rad
 */
int32_t get_rotation (int32_t last_right_motor_pos,int32_t last_left_motor_pos){
	return ((right_motor_get_pos()-last_right_motor_pos)-(left_motor_get_pos()-last_left_motor_pos))*180/TURN_STEP;
}

/**
 * @brief	converts a distance given in number of steps to a distance in cm
 *
 * @param 	nb_step 	distance in number of steps.
 * @return	distance in cm
 */
float step_to_cm (int32_t nb_step){
	return nb_step*WHEEL_PERIMETER*1./NSTEP_ONE_TURN;

}

float get_x() {
	return x;
}

float get_y() {
	return y;
}

int16_t get_angle() {
	return angle;
}

void robot_position_start(void){
	init_sin();
	init_cos();
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	chThdCreateStatic(waRobotPosition, sizeof(waRobotPosition), NORMALPRIO, RobotPosition, NULL);
}

