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



//////////Public functions//////////


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
