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

#include <motors.h>
#include <movements.h>

#define NSTEP_ONE_TURN      1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     13
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
	}
}

void init_cos (void){
	for (uint16_t i=0; i<MAX_ANGLE; i++){
		COS[i]=cosf(i*M_PI/180);
	}
}

static THD_WORKING_AREA(waRobotPosition, 256);
static THD_FUNCTION(RobotPosition, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	left_motor_set_pos(0);
	right_motor_set_pos(0);

	int32_t amplitude =0;

	while(1){
		chThdSleepMilliseconds(10);
		amplitude = get_translation();
		angle+= get_angle();
		angle %= MAX_ANGLE;
		if (angle<0){
			angle +=360;
		}

		x+=amplitude*COS[angle];
		y-=amplitude*SIN[angle];

		left_motor_set_pos(0);
		right_motor_set_pos(0);

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
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	if (angle>0){
		left_motor_set_speed(-speed);
		right_motor_set_speed(speed);
	}
	else {
		left_motor_set_speed(speed);
		right_motor_set_speed(-speed);
	}

	while(1)
	{
		if ((abs(left_motor_get_pos())>=abs((angle*TURN_STEP)/(2*M_PI))) &&
				(abs(right_motor_get_pos())>=abs((angle*TURN_STEP)/(2*M_PI))))
		{
			break;
		}

	}

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


<<<<<<< HEAD

void position_mode(float pos_r, float pos_l, float speed_r,  float speed_l)
{
	bool stop_r=false;
	bool stop_l=false;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	right_motor_set_speed(speed_r);
	left_motor_set_speed(speed_l);

	while (1)
	{
		if (abs(right_motor_get_pos())>=abs(pos_r*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
		{
			stop_r=true;
			right_motor_set_speed(0);
		}
		if (abs(left_motor_get_pos())>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
		{
			stop_l=true;
			left_motor_set_speed(0);
		}
		if (stop_r && stop_l) break;
	}

}


=======
/**
 * @brief	returns the translation of the epuck from the initalization point
 *
 * @return	the translation in cm
 */
float get_translation (void){
	return step_to_cm((left_motor_get_pos+right_motor_get_pos)/2);

}

/**
 * @brief	returns the angle of rotation of the epuck from the initalization point
 *
 * @return	the angle of rotation in rad
 */
int32_t get_rotation (void){
	return (-left_motor_get_pos+right_motor_get_pos)*180/TURN_STEP;
}

/**
 * @brief	converts a distance given in number of steps to a distance in cm
 *
 * @param 	nb_step 	distance in number of steps.
 * @return	distance in cm
 */
float step_to_cm (uint32_t nb_step){
	return nb_step*WHEEL_PERIMETER/TURN_STEP;

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
	chThdCreateStatic(waRobotPosition, sizeof(waRobotPosition), NORMALPRIO, RobotPosition, NULL);
}
>>>>>>> origin/main
