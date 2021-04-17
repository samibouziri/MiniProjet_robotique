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
#define TURN_STEP			((PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN) //nb steps for one full turn
#define PAS_VIT				1000
#define CLOSE_THR			100
#define EPSILON 			0.1f
static float x=0;
static float y=0;
static float angle=0;

static bool stop=0;
static bool arrived=0;
static bool changing_mode=0;


static BSEMAPHORE_DECL(detect_obstacle_sem, TRUE);

//static thread_t *Permission;
//static thread_t *Rotate;


//////////Private functions//////////

static THD_WORKING_AREA(waRobotPosition, 256);
static THD_FUNCTION(RobotPosition, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int32_t last_right_motor_pos=0;
	int32_t last_left_motor_pos=0;

	float amplitude =0;
	systime_t time=chVTGetSystemTime();

	chThdSleepMilliseconds(10);
	while(1){
		time=chVTGetSystemTime();
		amplitude = get_translation(last_right_motor_pos,last_left_motor_pos);
		angle+= get_rotation(last_right_motor_pos,last_left_motor_pos);
		if (angle >M_PI){
			angle -=2*M_PI;
		}
		if (angle <=-M_PI){
			angle +=2*M_PI;
		}
		//chprintf((BaseSequentialStream *)&SD3, "step1=%f\n\r",angle*180/M_PI);
		last_right_motor_pos=right_motor_get_pos();
		last_left_motor_pos=left_motor_get_pos();
		x+=amplitude*cosf(angle);
		y+=amplitude*sinf(angle);
		chprintf((BaseSequentialStream *)&SD3, "x: %f y: %f \n\r",x,y);
	//	chprintf((BaseSequentialStream *)&SD3, "angle: %f \n\r",get_angle());
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	//	 chprintf((BaseSequentialStream *)&SD3, "time: %d \n\r",chVTGetSystemTime()-time);


	}
}

bool allign_to_avoid (void){
	if (angle_colision()<0)
	{
		rotate_rad(M_PI/2+angle_colision(),600);
		return true;
	}
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),600);
		return false;
	}
}

bool collinear(float x1,float x2,float y1,float y2){
	if(abs(x1*y2-x2*y1)/sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))<EPSILON)
		return true;
	else
		return false;
}

void return_home(void){
	do
	{
		chBSemSignal(&detect_obstacle_sem);
		go_to_xy (0,0,600);
		if (!changing_mode){
			stop=false;
		}
		if(!arrived && !stop){
			float xs=x;
			float ys=y;
			float alpha=0;
			if (allign_to_avoid ()){
				do{
					turn_around_clockwise_speed();
					chThdSleepMilliseconds(15);
					alpha =atan2f((-y),(-x))-angle;
						if (alpha>M_PI)
							alpha -=2*M_PI;
						if (alpha<=-M_PI)
							alpha +=2*M_PI;
				}while (is_path_free(alpha) && !stop);
			}
			else{
				do{
					turn_around_anticlockwise_speed();
					chThdSleepMilliseconds(15);
					alpha =atan2f((-y),(-x))-angle;
					if (alpha>M_PI)
						alpha -=2*M_PI;
					if (alpha<=-M_PI)
						alpha +=2*M_PI;
				}while (is_path_free(alpha) && !stop);
			}
			arrived=false;
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}while (!arrived && !stop);
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
	if (!stop){
		arrived =0;
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

		while (!stop)
		{
			if (abs(right_motor_get_pos()-right_pos)>=abs(pos_r*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				//chprintf((BaseSequentialStream *)&SD3, "step=%d\n\r",abs(right_motor_get_pos()-right_pos));
				stop_r=true;
				right_motor_set_speed(0);
			}
			if (abs(left_motor_get_pos()-left_pos)>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				stop_l=true;
				left_motor_set_speed(0);
			}
			if (stop_r && stop_l) {
				arrived =1;
				break;
			}
		}
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}




/**
 * @brief	moves the robot forward with a certain speed*5.35
 *
 * @param 	distance (in cm) : distance to travel
 * @param 	speed ((in step/s): speed of travel

 */
void move_forward(float distance,  int16_t speed )
{
	position_mode(distance, distance, speed, speed);
}

void move_forward_speed( int16_t speed )
{
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

void turn_right( int16_t speed )
{
	right_motor_set_speed(speed/2);
	left_motor_set_speed(speed*2);
}


void rotate_left ( int16_t speed )
{
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
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
float get_rotation (int32_t last_right_motor_pos,int32_t last_left_motor_pos){
	return ((right_motor_get_pos()-last_right_motor_pos)-(left_motor_get_pos()-last_left_motor_pos))/WHEEL_DISTANCE*WHEEL_PERIMETER/NSTEP_ONE_TURN;
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

float get_angle() {
	return angle*180/M_PI;
}

void robot_position_start(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	chThdCreateStatic(waRobotPosition, sizeof(waRobotPosition), NORMALPRIO+1, RobotPosition, NULL);
}

void go_to_xy (float abscisse, float ordonnee, int16_t speed){
	float alpha =atan2f((ordonnee-y),(abscisse-x))-angle;
		if (alpha>M_PI)
			alpha -=2*M_PI;
		if (alpha<=-M_PI)
			alpha +=2*M_PI;
	rotate_rad( alpha, speed);
	move_forward( sqrt ((ordonnee-y)*(ordonnee-y)+(abscisse-x)*(abscisse-x)), speed );
}
/**
 * @brief	turns in circle around an obstacle clockwise
 *			in position mode
*/



void turn_around_clockwise_speed(void){
	if (!sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{

	//	right_motor_set_speed(PAS_VIT-600);
	//	left_motor_set_speed(PAS_VIT+600);
		right_motor_set_speed(400);
		left_motor_set_speed(1000);
//		chprintf((BaseSequentialStream *)&SD3, "soft right \r\n");
		return;
	}

	else if (sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR+40) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		if (sensor_close_obstacle(SENSOR_3,10*CLOSE_THR)){
			right_motor_set_speed(800);
			left_motor_set_speed(-800);
	//		chprintf((BaseSequentialStream *)&SD3, "2 \r\n");
			return;
		}
		right_motor_set_speed(600);
		left_motor_set_speed(600);
	//	chprintf((BaseSequentialStream *)&SD3, "forward \r\n");
		return;
	}
	else {
		left_motor_set_speed(-800);
		right_motor_set_speed(800);
	//	chprintf((BaseSequentialStream *)&SD3, "hard left \r\n");
	}


}

bool search_obstacle (void)
{
	while (!colision_detected())
	{
		right_motor_set_speed(800);
		left_motor_set_speed(800);
	}
	if (angle_colision()<0)
	{
		rotate_rad(M_PI/2+angle_colision(),600);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		return true;
	}
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),600);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		return false;
	}
}









void turn_around_anticlockwise_speed(void){
	if (!sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{

		//	right_motor_set_speed(PAS_VIT-600);
		//	left_motor_set_speed(PAS_VIT+600);
		right_motor_set_speed(1000);
		left_motor_set_speed(400);
		//		chprintf((BaseSequentialStream *)&SD3, "soft left \r\n");
		return;
	}

	else if (sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR+40)	)
	{
		if (sensor_close_obstacle(SENSOR_3,3*CLOSE_THR)){
			right_motor_set_speed(-800);
			left_motor_set_speed(800);
			//		chprintf((BaseSequentialStream *)&SD3, "2 \r\n");
			return;
		}
		right_motor_set_speed(600);
		left_motor_set_speed(600);
		//	chprintf((BaseSequentialStream *)&SD3, "forward \r\n");
		return;
	}
	else {
		left_motor_set_speed(800);
		right_motor_set_speed(-800);
		//	chprintf((BaseSequentialStream *)&SD3, "hard left \r\n");
	}


}
static THD_WORKING_AREA(waThdPermission, 1024);
static THD_FUNCTION(ThdPermission, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (1)
	{
		chBSemWait(&detect_obstacle_sem);
		while (!colision_detected())
		{
			chThdSleepMilliseconds(10);
		}
		stop=true;
	}
}

void set_stop (bool stop_value){
	stop =stop_value;
}

static THD_WORKING_AREA(waThdRotate, 1024);
static THD_FUNCTION(ThdRotate, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
	while(1)
	{
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		chBSemWait(&detect_obstacle_sem);
		time=chVTGetSystemTime();
		while(chVTGetSystemTime()<(time+MS2ST(5000)))
		{
			turn_around_clockwise_speed();
			chThdSleepMilliseconds(15);
		}

	}
}

void threads_start(void){
	chThdCreateStatic(waThdPermission, sizeof(waThdPermission), NORMALPRIO, ThdPermission, NULL);
//	chThdCreateStatic(waThdRotate, sizeof(waThdRotate), NORMALPRIO, ThdRotate, NULL);
}






