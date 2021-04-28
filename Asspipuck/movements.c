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
#include <sensors/proximity.h>
#include <process_image.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/VL53L0X/Api/core/inc/vl53l0x_api.h>

#define NSTEP_ONE_TURN      	1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      	5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     	(M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     	13.f
#define TURN_STEP				((PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN) //nb steps for one full turn
#define CLOSE_THR				100
#define MAX_DIST_MEAS			1200
#define TIME_OF_MEAS			400
#define DIST_IN_FRONT_OF_WALL	15
#define SPEED					600
#define TURN_INT_WHEEL_SPEED	400
#define TURN_EXT_WHEEL_SPEED	1000
#define HALT_SPEED				0
#define QUARTER_TURN 			M_PI/2
#define STEP_ROTATION 			M_PI/24
#define DEG_TO_RAD(deg)			deg*M_PI/180
#define MM_TO_CM(dist)			dist*0.1
#define APPROACH_ANGLE			73
#define PARALEL_TO_WALL_ANGLE	79
#define APPROACH_ANGLE1			75
#define PARALEL_TO_WALL_ANGLE1	81
#define ALLOWED_NUM_ERR			2
#define STABILIZATION_TIME		150
#define MIN_DETECTION_COUNT		2
#define MAX_DETECTION_COUNT		10
#define RECOGNITION_TURN_SPEED  200
#define WALL_DETECTED			800
#define THR_COEF				3
#define THR_BIAS				40
#define NUM_PARTS				5
#define MAX_ANGLE				M_PI
#define PERIOD					2*M_PI
#define ROBOT_POSITION_SLEEP	10
#define MAX_ANGLE_COLISION		M_PI/2

static float x=0;
static float y=0;
static float angle=0;

static bool stop=0;
static bool arrived=0;
static bool changing_mode=0;


static BSEMAPHORE_DECL(detect_obstacle_sem, TRUE);


//////////Private functions//////////

float confine_angle (float alpha){
	if (alpha>MAX_ANGLE)
		alpha -=PERIOD;
	if (alpha<=-MAX_ANGLE)
		alpha +=PERIOD;
	return alpha;
}

static THD_WORKING_AREA(waRobotPosition, 256);
static THD_FUNCTION(RobotPosition, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int32_t last_right_motor_pos=0;
	int32_t last_left_motor_pos=0;

	float amplitude =0;
	systime_t time=chVTGetSystemTime();

	chThdSleepMilliseconds(ROBOT_POSITION_SLEEP);
	while(1){
		time=chVTGetSystemTime();
		amplitude = get_translation(last_right_motor_pos,last_left_motor_pos);
		angle+= get_rotation(last_right_motor_pos,last_left_motor_pos);
		angle=confine_angle(angle);
		last_right_motor_pos=right_motor_get_pos();
		last_left_motor_pos=left_motor_get_pos();
		x+=amplitude*cosf(angle);
		y+=amplitude*sinf(angle);
		chThdSleepUntilWindowed(time, time + MS2ST(ROBOT_POSITION_SLEEP));

	}
}

bool allign_to_avoid (void){
	if (angle_colision()<0)
	{
		rotate_rad(M_PI/2+angle_colision(),SPEED);
		return true;
	}
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),SPEED);
		return false;
	}
}

void return_home(float xg, float yg){
	do
	{
		chBSemSignal(&detect_obstacle_sem);
		go_to_xy (xg,yg,SPEED);
		if (!changing_mode){
			stop=false;
		}
		if(!arrived && !stop){
			float alpha=0;
			if (allign_to_avoid ()){
				do{
					turn_around_clockwise_speed();
					chThdSleepMilliseconds(15);
					alpha =atan2f((yg-y),(xg-x))-angle;
					alpha=confine_angle (alpha);
				}while (!(alpha>M_PI/4 && alpha< 3*M_PI/4) && !stop);
			}
			else{
				do{
					turn_around_anticlockwise_speed();
					chThdSleepMilliseconds(15);
					alpha =atan2f((yg-y),(xg-x))-angle;
					alpha=confine_angle(alpha);
				}while (!(alpha>-3*M_PI/4 && alpha< -M_PI/4) && !stop);
			}
			arrived=false;
		}
		right_motor_set_speed(HALT_SPEED);
		left_motor_set_speed(HALT_SPEED);
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
	if (angle_colision>MAX_ANGLE_COLISION)
		angle_colision=MAX_ANGLE_COLISION;
	if (angle_colision<-MAX_ANGLE_COLISION)
		angle_colision=-MAX_ANGLE_COLISION;
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
				right_motor_set_speed(HALT_SPEED);
			}
			if (abs(left_motor_get_pos()-left_pos)>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				stop_l=true;
				left_motor_set_speed(HALT_SPEED);
			}
			if (stop_r && stop_l) {
				arrived =1;
				break;
			}
		}
	}
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
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
	left_motor_set_pos(HALT_SPEED);
	right_motor_set_pos(HALT_SPEED);
	chThdCreateStatic(waRobotPosition, sizeof(waRobotPosition), NORMALPRIO+1, RobotPosition, NULL);
}

void go_to_xy (float abscisse, float ordonnee, int16_t speed){
	for (uint8_t i=0; i<NUM_PARTS; i++){
		float alpha =atan2f((ordonnee-y),(abscisse-x))-angle;
		alpha=confine_angle(alpha);
		rotate_rad( alpha, speed);
		move_forward( sqrt ((ordonnee-y)*(ordonnee-y)+(abscisse-x)*(abscisse-x))/(NUM_PARTS-i), speed );
	}

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
		right_motor_set_speed(TURN_INT_WHEEL_SPEED);
		left_motor_set_speed(TURN_EXT_WHEEL_SPEED);
		return;
	}

	else if (sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR+THR_BIAS) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		if (sensor_close_obstacle(SENSOR_3,THR_COEF*CLOSE_THR)){
			right_motor_set_speed(SPEED);
			left_motor_set_speed(-SPEED);
			return;
		}
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
		return;
	}
	else {
		left_motor_set_speed(-SPEED);
		right_motor_set_speed(SPEED);
	}


}

bool search_wall (void)
{
	while (!colision_detected(WALL_DETECTED))
	{
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
	}
	stop=false;
	if (angle_colision()<0)
	{
		rotate_rad(DEG_TO_RAD(APPROACH_ANGLE1)+angle_colision(),SPEED);
		rotate_rad(DEG_TO_RAD(PARALEL_TO_WALL_ANGLE1)+angle_colision(),SPEED);
		return true;
	}
	else
	{
		rotate_rad(-DEG_TO_RAD(APPROACH_ANGLE1)+angle_colision(),SPEED);
		rotate_rad(-DEG_TO_RAD(PARALEL_TO_WALL_ANGLE1)+angle_colision(),SPEED);
		return false;
	}
}









void turn_around_anticlockwise_speed(void){
	if (!sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		right_motor_set_speed(TURN_EXT_WHEEL_SPEED);
		left_motor_set_speed(TURN_INT_WHEEL_SPEED);
		return;
	}

	else if (sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR+THR_BIAS)	)
	{
		if (sensor_close_obstacle(SENSOR_3,THR_COEF*CLOSE_THR)){
			right_motor_set_speed(-SPEED);
			left_motor_set_speed(SPEED);
			return;
		}
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
		return;
	}
	else {
		left_motor_set_speed(SPEED);
		right_motor_set_speed(-SPEED);
	}


}
static THD_WORKING_AREA(waThdPermission, 1024);
static THD_FUNCTION(ThdPermission, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (1)
	{
		chBSemWait(&detect_obstacle_sem);
		while (!colision_detected(200))
		{
			chThdSleepMilliseconds(10);
		}
		stop=true;
	}
}

void set_stop (bool stop_value){
	stop =stop_value;
}

void threads_start(void){
	chThdCreateStatic(waThdPermission, sizeof(waThdPermission), NORMALPRIO, ThdPermission, NULL);
}

/**
 * @brief	turn until it recognize 2 lines with the same width and
 * 			seperated by the width of one of them
 */
void turn_patern_recognition(void){
	uint8_t count =0;
	uint8_t err =ALLOWED_NUM_ERR;
	while  (count <MAX_DETECTION_COUNT){
		set_led(LED1,0);
		if (!get_detected()){
			if (count>=MIN_DETECTION_COUNT && err>0)
				err--;
			else {
				rotate_rad(STEP_ROTATION,RECOGNITION_TURN_SPEED);
				count=0;
				err=ALLOWED_NUM_ERR;
				set_led(LED1,1);
			}
		}
		count++;
		chThdSleepMilliseconds(STABILIZATION_TIME);
	}
}

/**
 * @brief	makes the e-puck go forward until it reaches a wall and
 * 			then place itself in front of it by a distance of 15 cm
 * 			and in the middle of it (assuming that there are walls
 * 			on its right and left)
 */
void calibration (void){

	uint16_t dist_d =0;
	uint16_t dist_g=0;
	uint16_t dist_a=0;
	set_stop (false);
	if(search_wall()){
		set_front_led(1);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_g=VL53L0X_get_dist_mm();
		}while (dist_g>MAX_DIST_MEAS);
		rotate_rad(-QUARTER_TURN, SPEED);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_a=VL53L0X_get_dist_mm();
		}while (dist_a>MAX_DIST_MEAS);
		rotate_rad(-DEG_TO_RAD(APPROACH_ANGLE)+angle_colision(),SPEED);
		rotate_rad(-DEG_TO_RAD(PARALEL_TO_WALL_ANGLE)+angle_colision(),SPEED);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_d=VL53L0X_get_dist_mm();
		}while (dist_d>MAX_DIST_MEAS);
		move_forward(MM_TO_CM((dist_d-(dist_g+dist_d)/2)),SPEED);
		rotate_rad(-QUARTER_TURN, SPEED);
		set_front_led(0);
	}
	else{
		set_front_led(1);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_d=VL53L0X_get_dist_mm();
		}while (dist_d>MAX_DIST_MEAS);
		rotate_rad(QUARTER_TURN, SPEED);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_a=VL53L0X_get_dist_mm();
		}while (dist_a>MAX_DIST_MEAS);
		rotate_rad(DEG_TO_RAD(APPROACH_ANGLE)+angle_colision(),SPEED);
		rotate_rad(DEG_TO_RAD(PARALEL_TO_WALL_ANGLE)+angle_colision(),SPEED);
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_g=VL53L0X_get_dist_mm();
		}while (dist_g>MAX_DIST_MEAS);
		move_forward(MM_TO_CM((dist_g-(dist_g+dist_d)/2)),SPEED);
		rotate_rad(QUARTER_TURN, SPEED);
		set_front_led(0);
	}
	move_forward(DIST_IN_FRONT_OF_WALL-MM_TO_CM(dist_a),SPEED);
	left_motor_set_pos(HALT_SPEED);
	right_motor_set_pos(HALT_SPEED);
}






