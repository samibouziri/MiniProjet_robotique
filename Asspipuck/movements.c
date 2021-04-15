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
#include <ir_sensor.h>

#define NSTEP_ONE_TURN      1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     13.f
#define TURN_STEP			(PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN //nb steps for one full turn
#define PAS_CM 				1
#define PAS_VIT				500

static float SIN[NSTEP_ONE_TURN];
static float COS[NSTEP_ONE_TURN];

static float x=0;
static float y=0;
static int16_t angle=0;

//////////Private functions//////////

void init_sin (void){
	for (uint16_t i=0; i<NSTEP_ONE_TURN; i++){
		SIN[i]=sinf(i*2*M_PI/NSTEP_ONE_TURN);
		chprintf((BaseSequentialStream *)&SD3, "sin[%d]=%f",i,SIN[i]);
	}
}

void init_cos (void){
	for (uint16_t i=0; i<NSTEP_ONE_TURN; i++){
		COS[i]=cosf(i*2*M_PI/NSTEP_ONE_TURN);
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
		angle %= NSTEP_ONE_TURN;
		if (angle<0){
			angle +=NSTEP_ONE_TURN;
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
void rotate_rad(float alpha, int16_t speed)
{
	position_mode(alpha*WHEEL_DISTANCE/2, -alpha*WHEEL_DISTANCE/2, abs(speed),  abs(speed));

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
/**
 * @brief	moves the right wheel by pos_r with a speed of speed_r
 * 			moves the left wheel by pos_l with a speed of speed_l
 *
 * @param 	pos_r right position (distance) to achieve (in cm)
 * @param 	pos_l left position (distance) to achieve (in cm)
 * @param 	speed_r speed of the right wheel (in step/s)
 * @param 	speed_l speed of the left wheel (in step/s)
 */

void position_mode(float pos_r, float pos_l, int16_t speed_r,  int16_t speed_l)
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
 * @brief	moves the robot forward with a certain speed
 *
 * @param 	distance (in cm) : distance to travel
 * @param 	speed ((in step/s): speed of travel

 */

void move_forward(float distance, uint16_t speed )
{
	position_mode(distance, distance, speed, speed);
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
	int16_t alpha =((right_motor_get_pos()-last_right_motor_pos)-(left_motor_get_pos()-last_left_motor_pos))/2%NSTEP_ONE_TURN;
	if (alpha<0)
		alpha +=NSTEP_ONE_TURN;
	return alpha;
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



void turn_around_clockwise(void)
{

	while(1)
	{
		if (!sensor_close_obstacle(SENSOR_1) && sensor_close_obstacle(SENSOR_3) )
			{
			move_forward(PAS_CM, PAS_VIT);
			continue;
			}
		if (sensor_close_obstacle(SENSOR_1) && sensor_close_obstacle(SENSOR_3) )
			{
			rotate_rad(M_PI/12, PAS_VIT);
			continue;

void turn_around_clockwise_speed(void)
{

		if ( (sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR)) && sensor_close_obstacle(SENSOR_3,CLOSE_THR) )
		{

			right_motor_set_speed(PAS_VIT+600);
			left_motor_set_speed(-PAS_VIT-600);
		//	position_mode(PAS_CM, 0, PAS_VIT, 0);
			chprintf((BaseSequentialStream *)&SD3, "1 \r\n");
			return;
		}
		if ( !(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR)) && !sensor_close_obstacle(SENSOR_3,CLOSE_THR) )
		{
			if (sensor_close_obstacle(SENSOR_2,CLOSE_THR))
				{
					right_motor_set_speed(PAS_VIT+600);
					left_motor_set_speed(-PAS_VIT-600);
					//	position_mode(PAS_CM, 0, PAS_VIT, 0);
					chprintf((BaseSequentialStream *)&SD3, "2 if \r\n");
					return;
				}

		/*	if (sensor_close_obstacle(SENSOR_7,CLOSE_THR))
					{
						right_motor_set_speed(PAS_VIT+600);
						left_motor_set_speed(-PAS_VIT-600);
						//	position_mode(PAS_CM, 0, PAS_VIT, 0);
						//	chprintf((BaseSequentialStream *)&SD3, "cas5 \r\n");
						continue;
					}
		*/
			right_motor_set_speed(PAS_VIT-600);
			left_motor_set_speed(PAS_VIT+600);
	//		position_mode(0, PAS_CM, 0 ,PAS_VIT );
	//		rotate_rad(-PAS_RAD, PAS_VIT);
			chprintf((BaseSequentialStream *)&SD3, "2 normal \r\n");
			return;
		}
		if ( (sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR)) && !sensor_close_obstacle(SENSOR_3,CLOSE_THR) )
		{
			right_motor_set_speed(PAS_VIT+600);
			left_motor_set_speed(-PAS_VIT-600);
	//		position_mode(PAS_CM, 0, PAS_VIT, 0);
			chprintf((BaseSequentialStream *)&SD3, "3 \r\n");
			return;
		}

		if (sensor_close_obstacle(SENSOR_2,CLOSE_THR))
			{
				right_motor_set_speed(PAS_VIT+600);
				left_motor_set_speed(-PAS_VIT-600);
				//	position_mode(PAS_CM, 0, PAS_VIT, 0);
				chprintf((BaseSequentialStream *)&SD3, "4 \r\n");
				return;
			}

		right_motor_set_speed(PAS_VIT);
		left_motor_set_speed(PAS_VIT);
		chprintf((BaseSequentialStream *)&SD3, "forward \r\n");

		if (sensor_close_obstacle(SENSOR_3,2*CLOSE_THR))
		{
			right_motor_set_speed(PAS_VIT+600);
			left_motor_set_speed(PAS_VIT-600);
			chprintf((BaseSequentialStream *)&SD3, "6 \r\n");
			return;
		}

		if (sensor_close_obstacle(SENSOR_7,CLOSE_THR))
		{
			right_motor_set_speed(PAS_VIT+600);
			left_motor_set_speed(-PAS_VIT-600);
			//	position_mode(PAS_CM, 0, PAS_VIT, 0);
				chprintf((BaseSequentialStream *)&SD3, "7 \r\n");
			return;
		}

}
		{
			rotate_rad(-M_PI/12, PAS_VIT);
			continue;
		}
		if (sensor_close_obstacle(SENSOR_1) && !sensor_close_obstacle(SENSOR_3) )
		{
			rotate_rad(M_PI/12, PAS_VIT);
			continue;
		}

	}

}



















