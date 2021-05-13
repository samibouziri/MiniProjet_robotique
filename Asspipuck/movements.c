/*	Project: AspiPuck
 * 	movements.c
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was created by the authors.
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
#include <tof.h>
#include <sensors/VL53L0X/Api/core/inc/vl53l0x_api.h>

#define NSTEP_ONE_TURN      	1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      	5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     	(M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     	13.f
#define TURN_STEP				((PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN) //nb steps for one full turn
#define CHARGING_FORWARD_STEP	4		//distance (in cm) the robot has to travel to completely pass the target
#define CHARGING_BACKWARD_STEP	5		//distance (in cm) the robot has to travel backward to start charging
#define CHARGING_ADVANCE_STEP	15 		//distance (in cm) the robot advance by after it has finished charging
#define CLOSE_THR				300
#define MAX_DIST_MEAS			1200
#define TIME_OF_MEAS			400
#define DIST_IN_FRONT_OF_WALL	15
#define SPEED					700
#define TURN_INT_WHEEL_SPEED	350
#define TURN_EXT_WHEEL_SPEED	1000
#define HALT_SPEED				0
#define QUARTER_TURN 			M_PI/2
#define STEP_ROTATION 			M_PI/24
#define DEG_TO_RAD(deg)			(deg*M_PI)/180
#define MM_TO_CM(dist)			dist*0.1
#define APPROACH_ANGLE			73
#define PARALEL_TO_WALL_ANGLE	79
#define APPROACH_ANGLE1			76
#define PARALEL_TO_WALL_ANGLE1	82
#define ALLOWED_NUM_ERR			2
#define STABILIZATION_TIME		150
#define MIN_DETECTION_COUNT		2
#define MAX_DETECTION_COUNT		10
#define RECOGNITION_TURN_SPEED  200
#define WALL_DETECTED			800
#define OBSTACLE_THR 			700
#define ROTATE_CLOSE_THR		1200
#define ROTATE_SPEED			600
#define AVOID_SPEED				800
#define AVOID_MEDIUM_SPEED		300
#define AVOID_LOW_SPEED			120
#define FRONT_THR_CLOSE			900
#define FRONT_THR_MEDIUM		400
#define FRONT_THR_FAR			100
#define THR_CHARGING			200
#define SENSOR_LOW_THR			40
#define THR_COEF				3
#define FORWARD_COEF			3/4
#define THR_BIAS				40
#define NUM_PARTS				5
#define MAX_ANGLE				M_PI
#define PERIOD					2*M_PI
#define ROBOT_POSITION_SLEEP	10
#define MAX_ANGLE_COLISION		M_PI/2
#define MINIMAL_ANGLE_EXIT		M_PI/4
#define MAXIMAL_ANGLE_EXIT		3*M_PI/4
#define COLLISION_SLEEP			10		//sleeping time during the detection of a collision
#define GO_AND_AVOID_SLEEP		15		//sleeping time for the rotation around the obstacle
#define SOFT_CLEANING_SLEEP		100		//sleeping time in the soft cleaning mode to avoid detecting the same collision twice
#define DEEP_CLEANING_SLEEP		10		//sleeping time betwine each step when the robot is in deep cleaning mode
#define CHARGING_SLEEP			5		//sleeping time for the robot after it has finished charging
#define RETURN_HOME_SLEEP		5		//sleeping time for the robot after it has returned home
#define STOP_SLEEP				5		//sleeping time for the robot after it has finished halt
#define CHARGING_FORWARD_SLEEP	10		//sleeping time betwwen each step when the robot is advancing parallel to the wall
#define BLINKING_TIME			500		//blinking period of the leds when the robot is charging
#define FULLY_CHARGED_TIME		1000	//blinking time of the led after the robot has fully charger
#define NUMBER_OF_BLINKS		5
#define BLUE					0,0,255
#define RED						255,0,0
#define GREEN					0,255,0
#define	WHITE					255,255,255
#define ORANGE					255,165,0
#define HOME_POS				0,0

static float x=0;
static float y=0;
static float angle=0;
static bool stop=false;
static bool arrived=false;
static bool changing_mode=false;
static mode_puck_t mode =HALT;
static bool calibrating=false;
static bool allow_arrived=false;

static BSEMAPHORE_DECL(detect_obstacle_sem, TRUE);


//////////Private functions//////////


/**
 * @brief	confine the angle in the range of [-pi,pi]
 *
 * @param 	angle (in rad):	angle that is out of range of [-pi,pi]
 * @return	angle in the main angles' range
 */
float confine_angle (float alpha){
	if (alpha>MAX_ANGLE)
		alpha -=PERIOD;
	if (alpha<=-MAX_ANGLE)
		alpha +=PERIOD;
	return alpha;
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
	//the static variable stop can be set to true(ie: robot is in front of an obstacle/ reached
	//his goal position..) if so, this function doesn't move the robot.
	if (!stop){
		//if the robot doesn't need to stop, then it still hasn't reached his goal -> arrived is set
		// to false.
		arrived =false;
		//to stop each wheel independently if they have different travel distances
		bool stop_r=false;
		bool stop_l=false;
		//set the starting position of each wheel before moving
		int32_t right_pos =right_motor_get_pos();
		int32_t left_pos =left_motor_get_pos();
		//if the given goal position is negative, the wheel goes backward
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
		//if the robot needs to stop midway before reaching his goal (ie there is an obstacle ahead
		//or the operating mode has changed)
		while (!stop)
		{
			//if the right wheel has reached its goal -> stop the right wheel
			if (abs(right_motor_get_pos()-right_pos)>=abs(pos_r*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				stop_r=true;
				right_motor_set_speed(HALT_SPEED);
			}
			//if the left wheel has reached its goal -> stop the left wheel
			if (abs(left_motor_get_pos()-left_pos)>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				stop_l=true;
				left_motor_set_speed(HALT_SPEED);
			}
			//both wheels reached their goal->the robot has reached his goal position
			if (stop_r && stop_l) {
				if (allow_arrived)
				{
					arrived =true;
				}
				break;
			}
		}
	}
	//stop the motors if the robot reached its goal or if it need to stop for any other reason (ie the
	//operating mode has changed, the robot is facing an obstacle...)
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
}


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
 * @brief	rotate the e-puck to the right or to the left depending on the angle
 *			of collision to get it parallel with the obstacle
 *
 * @return	true if rotation done to the left, false if rotation done to the right
 */
bool allign_to_avoid (void){
	//rotate to the left if the obstacle is on the right of the robot
	if (angle_colision()<0)
	{
		rotate_rad(M_PI/2+angle_colision(),SPEED);
		return true;
	}
	//rotate to the right if the obstacle is on the left of the robot
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),SPEED);
		return false;
	}
}



/**
 * @brief	for a given angle of collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
float angle_reflection (float angle_colision){
	//confine the angle of collision if the calculated value is off limits
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
 * @brief	moves the robot by a certain distance with a certain speed
 *
 * @param 	distance (in cm) : distance to travel
 * @param 	speed (in step/s): speed of travel

 */
void move_forward(float distance,  int16_t speed )
{
	position_mode(distance, distance, speed, speed);
}

/**
 * @brief	moves the robot forward with a certain speed
 *
 * @param 	speed (in step/s): speed of travel
 */
void move_forward_speed( int16_t speed )
{
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

/**
 * @brief	turn the robot to the right with a certain speed
 *
 * @param 	speed (in step/s): speed of travel
 */
void turn_right( int16_t speed )
{
	right_motor_set_speed(speed/2);
	left_motor_set_speed(speed*2);
}

/**
 * @brief	rotate the robot to the left with a certain speed
 *
 * @param 	speed (in step/s): speed of rotation
 */
void rotate_left ( int16_t speed )
{
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
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



/**
 * @brief	returns the translation of the epuck from the initalization point
 *
 * @return	the translation in cm
 */
float get_translation (int32_t last_right_motor_pos,int32_t last_left_motor_pos){
	return step_to_cm(((left_motor_get_pos()-last_left_motor_pos)+(right_motor_get_pos()-last_right_motor_pos))/2);

}

/**
 * @brief	returns the angle of rotation of the epuck from the initialization point
 *
 * @return	the angle of rotation in rad
 */
float get_rotation (int32_t last_right_motor_pos,int32_t last_left_motor_pos){
	return ((right_motor_get_pos()-last_right_motor_pos)-(left_motor_get_pos()-last_left_motor_pos))/WHEEL_DISTANCE*WHEEL_PERIMETER/NSTEP_ONE_TURN;
}



/**
 * @brief	returns the x coordinate of the robot
 *
 *
 * @return	coordinate x in cm
 */
float get_x(void) {
	return x;
}

/**
 * @brief	returns the y coordinate of the robot
 *
 * @return	coordinate y in cm
 */
float get_y(void) {
	return y;
}

/**
 * @brief	returns the angle of rotation of the robot compared to
 * 			the starting position
 *
 * @return	angle of rotation in degrees.
 */
float get_angle(void) {
	return angle*180/M_PI;
}


/**
 * @brief	go to the designed coordinates with a certain speed
 * 			starting from the current position
 *
 * @param	abscisse (in cm) : abscissa of the goal position
 * @param	ordonnee (in cm) : ordinate of the goal position
 * @param	speed (in step/s): speed of travel
 */
void go_to_xy (float abscisse, float ordonnee, int16_t speed){
	// divide the path into multiple paths with small travel distance to minimise the error
	for (uint8_t i=0; i<NUM_PARTS; i++){
		float alpha =atan2f((ordonnee-y),(abscisse-x))-angle;
		alpha=confine_angle(alpha);
		rotate_rad( alpha, speed);
		move_forward( sqrt ((ordonnee-y)*(ordonnee-y)+(abscisse-x)*(abscisse-x))/(NUM_PARTS-i), speed );
	}

}

/**
 * @brief	turns in circles clockwise around an obstacle
 *
 */
void turn_around_clockwise_speed(void){

	//no detection -> away from obstacle -> go closer to obstacle
	if (!sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR+200) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		right_motor_set_speed(TURN_INT_WHEEL_SPEED);
		left_motor_set_speed(TURN_EXT_WHEEL_SPEED);
		return;
	}
	//only sensor 3 detects -> robot is close to the obstacle
	else if (sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,get_calibrated_prox(SENSOR_3)*FORWARD_COEF) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		//robot too close to obstacle -> in place rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_3,ROTATE_CLOSE_THR)){
			right_motor_set_speed(TURN_EXT_WHEEL_SPEED);
			left_motor_set_speed(TURN_INT_WHEEL_SPEED);
			return;
		}
		//robot close to obstacle -> rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_3,THR_COEF*CLOSE_THR)){
			right_motor_set_speed(AVOID_SPEED);
			left_motor_set_speed(HALT_SPEED);
			return;
		}
		//robot is at the right distance from the wall -> go forward
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
		return;
	}
	else {
		//obstacle in front of the robot -> rotate depending on the distance to the obstacle:

		//robot too close to obstacle -> in place rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_1,FRONT_THR_CLOSE)||
			sensor_close_obstacle(SENSOR_8,FRONT_THR_CLOSE)||
			sensor_close_obstacle(SENSOR_7,FRONT_THR_CLOSE)){
			right_motor_set_speed(ROTATE_SPEED);
			left_motor_set_speed(-ROTATE_SPEED);
		}
		//robot close to obstacle -> rotate to avoid collision
		else if (sensor_close_obstacle(SENSOR_1,FRONT_THR_MEDIUM)||
				sensor_close_obstacle(SENSOR_8,FRONT_THR_MEDIUM)||
				sensor_close_obstacle(SENSOR_7,FRONT_THR_MEDIUM)){
			right_motor_set_speed(ROTATE_SPEED);
			left_motor_set_speed(-AVOID_MEDIUM_SPEED);
		}
		//robot heading towards wall but still not too close-> small rotation to prepare in advance for
		//the avoidance of the incoming obstacle
		else if (sensor_close_obstacle(SENSOR_1,FRONT_THR_FAR)||
				sensor_close_obstacle(SENSOR_8,FRONT_THR_FAR)||
				sensor_close_obstacle(SENSOR_7,FRONT_THR_FAR)){
			right_motor_set_speed(AVOID_SPEED);
			left_motor_set_speed(-AVOID_LOW_SPEED);
		}
		//robot too close diagonally to an obstacle -> in place rotation to avoid collision
		else if (sensor_close_obstacle(SENSOR_2,ROTATE_CLOSE_THR)){
			right_motor_set_speed(ROTATE_SPEED);
			left_motor_set_speed(-ROTATE_SPEED);
			return;
		}else{
			//sums all the cases where the robot needs to avoid an obstacle (surrounded by walls, dead end, etc..)
			//lowers the threshold on sensor 2 -> make the robot prone to rotate left.
			left_motor_set_speed(-AVOID_LOW_SPEED);
			right_motor_set_speed(AVOID_SPEED);
		}

	}
}



/**
 * @brief	goes forward while no obstacle is detected, then align the robot
 * 			if in front of an obstacle
 *
 * @return	true if the robot aligned itself to the left and false if the robot
 * 			aligned itself to the right.
 */
bool search_wall (void)
{
	//go forward if no obstacle is ahead
	while (!colision_detected(WALL_DETECTED))
	{
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
	}
	stop=false;
	//rotate to the left if the obstacle is on the right of the robot
	if (angle_colision()<0)
	{
		rotate_rad(DEG_TO_RAD(APPROACH_ANGLE1)+angle_colision(),SPEED);
		rotate_rad(DEG_TO_RAD(PARALEL_TO_WALL_ANGLE1)+angle_colision(),SPEED);
		return true;
	}
	//rotate to the right if the obstacle is on the left of the robot
	else
	{
		rotate_rad(-DEG_TO_RAD(APPROACH_ANGLE1)+angle_colision(),SPEED);
		rotate_rad(-DEG_TO_RAD(PARALEL_TO_WALL_ANGLE1)+angle_colision(),SPEED);
		return false;
	}
}


/**
 * @brief	go forward while no obstacle is detected while the robot is in
 * 			deep cleaning mode, then align the robot if in front of an obstacle
 *
 * @return	true if the robot aligned itself to the left and false if the robot
 * 			aligned itself to the right.
 */
bool search_obstacle_turn (void)
{
	//go forward if no obstacle is ahead
	while (!colision_detected(CLOSE_THR ) && mode==DEEP_CLEANING )
	{
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
	}
	//rotate to the left if the obstacle is on the right of the robot
	if (angle_colision()<0)
	{
		rotate_rad(M_PI/2+angle_colision(),SPEED);
		return true;
	}
	//rotate to the right if the obstacle is on the left of the robot
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),SPEED);
		return false;
	}
}


/**
 * @brief	turns in circles anticlockwise around an obstacle
 *
 */
void turn_around_anticlockwise_speed(void){

	//no detection -> away from obstacle -> go closer to obstacle
	if (!sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_8,CLOSE_THR)||sensor_close_obstacle(SENSOR_1,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR)	)
	{
		left_motor_set_speed(TURN_INT_WHEEL_SPEED);
		right_motor_set_speed(TURN_EXT_WHEEL_SPEED);
		return;
	}
	//only sensor 6 detects -> robot is close to the obstacle
	else if (sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_7,get_calibrated_prox(SENSOR_6)*FORWARD_COEF) &&
			!(sensor_close_obstacle(SENSOR_8,CLOSE_THR)||sensor_close_obstacle(SENSOR_1,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR)	)
	{
		//robot too close to obstacle -> in place rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_6,ROTATE_CLOSE_THR)){
			right_motor_set_speed(-ROTATE_SPEED);
			left_motor_set_speed(ROTATE_SPEED);
			return;
		}
		//robot close to obstacle -> rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_6,THR_COEF*CLOSE_THR)){
			left_motor_set_speed(AVOID_SPEED);
			right_motor_set_speed(HALT_SPEED);
			return;
		}
		//robot is at the right distance from the wall -> go forward
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
		return;
	}
	else {
		//obstacle in front of the robot -> rotate depending on the distance to the obstacle:

		//robot too close to obstacle -> in place rotation to avoid collision
		if (sensor_close_obstacle(SENSOR_8,FRONT_THR_CLOSE)||
			sensor_close_obstacle(SENSOR_1,FRONT_THR_CLOSE)||
			sensor_close_obstacle(SENSOR_2,FRONT_THR_CLOSE)){
			right_motor_set_speed(-ROTATE_SPEED);
			left_motor_set_speed(ROTATE_SPEED);
		}
		//robot close to obstacle -> rotate to avoid collision
		else if (sensor_close_obstacle(SENSOR_8,FRONT_THR_MEDIUM)||
				sensor_close_obstacle(SENSOR_1,FRONT_THR_MEDIUM)||
				sensor_close_obstacle(SENSOR_2,FRONT_THR_MEDIUM)){
			right_motor_set_speed(-AVOID_MEDIUM_SPEED);
			left_motor_set_speed(ROTATE_SPEED);
		}
		//robot heading towards wall but still not too close-> small rotation to prepare in advance for
		//the avoidance of the incoming obstacle
		else if (sensor_close_obstacle(SENSOR_8,FRONT_THR_FAR)||
				sensor_close_obstacle(SENSOR_1,FRONT_THR_FAR)||
				sensor_close_obstacle(SENSOR_2,FRONT_THR_FAR)){
			right_motor_set_speed(-AVOID_LOW_SPEED);
			left_motor_set_speed(AVOID_SPEED);
		}
		//robot too close diagonally to an obstacle -> in place rotation to avoid collision
		else if (sensor_close_obstacle(SENSOR_7,ROTATE_CLOSE_THR)){
			right_motor_set_speed(-ROTATE_SPEED);
			left_motor_set_speed(ROTATE_SPEED);
			return;
		}else{
			//sums all the cases where the robot needs to avoid an obstacle (surrounded by walls, dead end, etc..)
			//lowers the threshold on sensor 7 -> make the robot prone to rotate right.
			left_motor_set_speed(AVOID_SPEED);
			right_motor_set_speed(-AVOID_LOW_SPEED);
		}

	}
}



/**
 * @brief	sets the rgb leds with the color corresponding
 * 			to the halt mode
 */
void set_rgb_halt(void)
{
	set_rgb_led(LED2,RED);
	set_rgb_led(LED4,RED);
	set_rgb_led(LED6,RED);
	set_rgb_led(LED8,RED);
}



/**
 * @brief	sets the rgb leds with the color corresponding
 * 			to the ricochet mode
 */
void set_rgb_soft_cleaning(void)
{
	set_rgb_led(LED2,BLUE);
	set_rgb_led(LED4,BLUE);
	set_rgb_led(LED6,BLUE);
	set_rgb_led(LED8,BLUE);
}



/**
 * @brief	sets the rgb leds with the color corresponding
 * 			to the turn_around mode
 */
void set_rgb_deep_cleaning(void)
{
	set_rgb_led(LED2,WHITE);
	set_rgb_led(LED4,WHITE);
	set_rgb_led(LED6,WHITE);
	set_rgb_led(LED8,WHITE);
}



/**
 * @brief	sets the rgb leds with the color corresponding
 * 			to the go_home mode
 */
void set_rbg_return_home(void)
{
	set_rgb_led(LED2,GREEN);
	set_rgb_led(LED4,GREEN);
	set_rgb_led(LED6,GREEN);
	set_rgb_led(LED8,GREEN);
}

/**
 * @brief	sets the rgb leds with the color corresponding
 * 			to the charging_mode
 */

void set_rbg_charging(void)
{
	set_rgb_led(LED2,ORANGE);
	set_rgb_led(LED4,ORANGE);
	set_rgb_led(LED6,ORANGE);
	set_rgb_led(LED8,ORANGE);
}


/**
 * @brief	commands the e-puck to go to a certain location and stop there
 * 			when the location is reached. The e-puck avoids all obstacles
 * 			while going to the designed location.
 *
 * @param 	xg:	abscissa of the goal locaztion (in cm)
 * @param	yg:	ordinate of the goal locaztion (in cm)
 */
void go_and_avoid(float xg, float yg){
	// allow the change of the variable arrived and to stop the robot if
	// an obstacle is detected
	allow_arrived=true;

	do
	{
		//allow the detection of collisions
		chBSemSignal(&detect_obstacle_sem);
		//go to the goal coordinates
		go_to_xy (xg,yg,SPEED);
		//reset the stop variable if the robot is not changing the mode
		if (!changing_mode)
		{
			stop=false;
		}
		//verify that the robot hasn't arrived yet and the we are not changing modes
		if(!arrived && !stop){
			float alpha=0;

			//allign the robot perpendicular to the wall and decide if it should turn
			//clockwise or anticlockwise
			if (allign_to_avoid ()){
				do{
					turn_around_clockwise_speed();
					//let the robot go forward a bit
					chThdSleepMilliseconds(GO_AND_AVOID_SLEEP);

					//caculate the angle between the orientation of the robot and the goal
					//coordinates to determine if it can leave or not
					alpha =atan2f((yg-y),(xg-x))-angle;
					alpha=confine_angle(alpha);

					//verify if the robot should leave the obstacle or should stop
				}while (!(alpha>MINIMAL_ANGLE_EXIT && alpha< MAXIMAL_ANGLE_EXIT) && !stop);
			}
			else{
				do{
					turn_around_anticlockwise_speed();

					//let the robot go forward a bit
					chThdSleepMilliseconds(GO_AND_AVOID_SLEEP);

					//caculate the angle between the orientation of the robot and the goal
					//coordinates to determine if it can leave or not
					alpha =atan2f((yg-y),(xg-x))-angle;
					alpha=confine_angle(alpha);

					//verify if the robot should leave the obstacle or should stop
				}while (!(alpha>-MAXIMAL_ANGLE_EXIT && alpha<-MINIMAL_ANGLE_EXIT) && !stop);
			}
			arrived=false;
		}
		//stop the motors
		right_motor_set_speed(HALT_SPEED);
		left_motor_set_speed(HALT_SPEED);

		//exit if the robot arrived to destination or it is told to stop
	}while (!arrived && !stop);

	//stop the change the variable arrived
	allow_arrived= false;
}



/**
 * @brief	turn until it recognize 2 lines with the same width and
 * 			seperated by the width of one of them
 */
void turn_patern_recognition(void){

	//start the processing of the images
	process_image_start();
	uint8_t count =0;
	uint8_t err =ALLOWED_NUM_ERR;

	// while the patern isn't recognized at least MAX_DETECTION_COUNT
	// times with ALLOWED_NUM_ERR number of allowed errors to avoid
	// recognizing noise the robot should blink the led1 and turn and
	// stop to stabilize the image
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

	//stop the process of the image
	process_image_stop();
}

/**
 * @brief	makes the e-puck go forward until it reaches a wall and
 * 			then place itself in front of it by a distance of 15 cm
 * 			and in the middle of it (assuming that there are walls
 * 			on its right and left)
 */
void calibration (void){
	// start the TOF sensor
	TOF_start();
	uint16_t dist_d =0;
	uint16_t dist_g=0;
	uint16_t dist_a=0;
	set_stop (false);
	// go forward until the robot reaches a wall and turn precisely perpendicular to the wall
	if(search_wall()){
		//anticlockwise sequence
		set_front_led(1);
		// get the distance to the left wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_g=TOF_get_dist_mm();
		}while (dist_g>MAX_DIST_MEAS);
		//turn to the back wall
		rotate_rad(-QUARTER_TURN, SPEED);
		// get the distance to the back wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_a=TOF_get_dist_mm();
		}while (dist_a>MAX_DIST_MEAS);
		//turn to the right wall (and insure the robot is perpendicular to the wall )
		rotate_rad(-DEG_TO_RAD(APPROACH_ANGLE)+angle_colision(),SPEED);
		rotate_rad(-DEG_TO_RAD(PARALEL_TO_WALL_ANGLE)+angle_colision(),SPEED);
		// get the distance to the right wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_d=TOF_get_dist_mm();
		}while (dist_d>MAX_DIST_MEAS);
		//recenter the robot according to the right and left wall
		move_forward(MM_TO_CM((dist_d-(dist_g+dist_d)/2)),SPEED);
		//face forward
		rotate_rad(-QUARTER_TURN, SPEED);
		set_front_led(0);
	}
	else{
		//clockwise sequence

		set_front_led(1);
		// get the distance to the right wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_d=TOF_get_dist_mm();
		}while (dist_d>MAX_DIST_MEAS);
		//turn to the back wall
		rotate_rad(QUARTER_TURN, SPEED);
		// get the distance to the back wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_a=TOF_get_dist_mm();
		}while (dist_a>MAX_DIST_MEAS);
		//turn to the left wall (and insure the robot is perpendicular to the wall )
		rotate_rad(DEG_TO_RAD(APPROACH_ANGLE)+angle_colision(),SPEED);
		rotate_rad(DEG_TO_RAD(PARALEL_TO_WALL_ANGLE)+angle_colision(),SPEED);
		// get the distance to the left wall (recalculate until the distance seems reasonable (<1.2m))
		do{
			chThdSleepMilliseconds(TIME_OF_MEAS);
			dist_g=TOF_get_dist_mm();
		}while (dist_g>MAX_DIST_MEAS);
		//recenter the robot according to the right and left wall
		move_forward(MM_TO_CM((dist_g-(dist_g+dist_d)/2)),SPEED);
		//face forward
		rotate_rad(QUARTER_TURN, SPEED);
		set_front_led(0);
	}
	//move away from the nearest wall by a certain distance
	move_forward(DIST_IN_FRONT_OF_WALL-MM_TO_CM(dist_a),SPEED);
	//reinitialize the cordinates and the angle
	x=0;
	y=0;
	angle=0;
	// stop the TOF sensor
	TOF_stop();
}


/**
 * @brief	main function to operate the e-puck in the charging mode
 */
void charging (void){
	//flag set to not interrupt the robot while calibrating
	calibrating=true;
	// returnnig to home position
	go_and_avoid(HOME_POS);
	// the robot turn around itself until it recognize a wall with a patern
	turn_patern_recognition();
	//go forward until it reaches the wall
	bool direction= search_wall();
	// follow the wall until it reaches its end
	if(direction){
		while (sensor_close_obstacle(SENSOR_3,THR_CHARGING)){
			move_forward_speed(SPEED);
			chThdSleepMilliseconds(CHARGING_FORWARD_SLEEP);
		}
		//go forward a bit further to not hit the wall when reverse
		move_forward(CHARGING_FORWARD_STEP,SPEED);
		//rotate to face forward
		rotate_rad(QUARTER_TURN, SPEED);

	}else{
		while (sensor_close_obstacle(SENSOR_6,THR_CHARGING)){
			move_forward_speed(SPEED);
			chThdSleepMilliseconds(CHARGING_FORWARD_SLEEP);
		}
		//go forward a bit further to not hit the wall when reverse
		move_forward(CHARGING_FORWARD_STEP,SPEED);
		//rotate to face forward
		rotate_rad(-QUARTER_TURN, SPEED);
	}
	//reverse to enter the charging spot
	move_forward(-CHARGING_BACKWARD_STEP,SPEED);

	uint8_t count=0;
	//blinking the leds to indicate that it is charging
	while (count <NUMBER_OF_BLINKS){
		set_rbg_charging();
		chThdSleepMilliseconds(BLINKING_TIME);
		clear_leds();
		chThdSleepMilliseconds(BLINKING_TIME);
		count ++;
	}
	//set body led to indicate that the charging is complete
	set_body_led(1);
	chThdSleepMilliseconds(FULLY_CHARGED_TIME);
	set_body_led(0);
	//move forward to be able to see the patern
	move_forward(CHARGING_ADVANCE_STEP,SPEED);
	// turn to speed up the patern recognition phase
	if (direction){
		rotate_rad(QUARTER_TURN, SPEED);
	}else{
		rotate_rad(2*QUARTER_TURN, SPEED);
	}

	turn_patern_recognition();
	calibration();
	calibrating=false;

}



/**
 * @brief	main function used to operate the e-puck in soft cleaning mode.
 * 			In this mode the e-puck will ricochet on obstacles with an angle of
 * 			reflection equal to the angle of collision.
 */
void soft_cleaning(void)
{
	bool step=false;
	while (mode==SOFT_CLEANING)
	{
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
		// if the robot has just turn it should go forward a little bit so
		// that it doesn't detect the obstacle again
		if (mode==SOFT_CLEANING && step) {
			chThdSleepMilliseconds(SOFT_CLEANING_SLEEP);
			step=false;
		}
		// if a obstacle was detected turn by the right amount
		if (mode==SOFT_CLEANING && colision_detected(WALL_DETECTED ))
		{
			rotate_rad(angle_reflection (angle_colision()),SPEED);
			step=true;
		}
	}
	//stop motors and exit
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
}

/**
 * @brief	operates the e-puck in deep cleaning mode: in this mode, the
 * 			e-puck goes forward while no obstacle is detected then starts
 * 			circling around the first detected obstacle in a clockwise
 * 			or anticlockwise direction depending on the angle of collision
 */
void deep_cleaning(void)
{
	//	goes forward until it reaches an obstacle and turn perpendicular to
	// it to prepare turning around around it and deciding if the robot should
	// turn clockwise or anticlockwise
	bool turn_clockwise=search_obstacle_turn();
	//turning around the obstacle
	while (mode==DEEP_CLEANING)
	{
		if (turn_clockwise)
		{
			turn_around_clockwise_speed();
		}
		else if (!turn_clockwise)
		{
			turn_around_anticlockwise_speed();
		}
		chThdSleepMilliseconds(DEEP_CLEANING_SLEEP);
	}
	//stopping the motors when finished
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
	return;
}


/**
 * @brief	operates the e-puck in halt mode: sets the motor's speed
 * 			to zero while in this mode
 */
void halt_mode(void)
{
	while (mode==HALT){
		//stop motors
		right_motor_set_speed(HALT_SPEED);
		left_motor_set_speed(HALT_SPEED);
		chThdSleepMilliseconds(STOP_SLEEP);
	}
}






/*
 * Thread: this thread is responsible for tracking the position of the robot
 * while he is moving. It runs constantly in the background from the moment the
 * robot is turned on.
 */
static THD_WORKING_AREA(waRobotPosition, 1024);
static THD_FUNCTION(RobotPosition, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int32_t last_right_motor_pos=0;
	int32_t last_left_motor_pos=0;

	float amplitude =0;
	systime_t time=chVTGetSystemTime();

	chThdSleepMilliseconds(ROBOT_POSITION_SLEEP);
	while(1){
		//capture time
		time=chVTGetSystemTime();
		amplitude = get_translation(last_right_motor_pos,last_left_motor_pos);
		//update the angle of rotation of the robot
		angle+= get_rotation(last_right_motor_pos,last_left_motor_pos);
		//making sure it doesn't exceed pi or go below -pi
		angle=confine_angle(angle);
		//updating the last position of the motors for the next measurement
		last_right_motor_pos=right_motor_get_pos();
		last_left_motor_pos=left_motor_get_pos();
		// update the coordinates of the robot
		x+=amplitude*cosf(angle);
		y+=amplitude*sinf(angle);
		//update the position every ROBOT_POSITION_SLEEP miiliseconds
		chThdSleepUntilWindowed(time, time + MS2ST(ROBOT_POSITION_SLEEP));

	}
}


/*
 * Thread: this thread is responsible for checking if the robot is in contact
 * with an obstacle when returning home. If yes the robot can go around the
 * obstacle and the thread will start checking for the next collision.
 */
static THD_WORKING_AREA(waThdCollision, 1024);
static THD_FUNCTION(ThdCollision, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (true)
	{
		//wait until the need to detect an obstacle
		chBSemWait(&detect_obstacle_sem);
		while (!colision_detected(OBSTACLE_THR ) && allow_arrived)
		{
			//obstacle not detected
			chThdSleepMilliseconds(COLLISION_SLEEP);
		}
		// an obstacle was found and there is still
		// the need to detect an obstacle
		if (allow_arrived) {
			stop=true;
		}
	}
}


////////////////////////////////////////////////////Public functions/////////////////////////////////////



/**
 * @brief	operate the e-puck in the mode corresponding to the state
 * 			of the variable mode
 */
void operating_mode(void)
{
	while(1) {

		changing_mode=false;
		stop=false;
		switch (mode)
		{
		case HALT:
			set_rgb_halt();
			halt_mode();
			clear_leds();
			break;
		case SOFT_CLEANING:
			set_rgb_soft_cleaning();
			soft_cleaning();
			clear_leds();
			break;
		case DEEP_CLEANING:
			set_rgb_deep_cleaning();
			deep_cleaning();
			clear_leds();
			break;
		case RETURN_HOME :
			set_rbg_return_home();
			go_and_avoid(HOME_POS);
			//calibration sequence
			if (!stop){
				calibrating=true;
				turn_patern_recognition();
				calibration ();
				calibrating=false;
			}
			// return home finished and waiting for a new mode
			while (mode==RETURN_HOME){
				chThdSleepMilliseconds(RETURN_HOME_SLEEP);
			}
			clear_leds();
			break;
		case CHARGING :
			set_rbg_charging();
			charging ();
			set_body_led(1);
			// charging finished and waiting for a new mode
			while (mode==CHARGING){
				chThdSleepMilliseconds(CHARGING_SLEEP);
			}
			set_body_led(0);
			break;
		}
	}
}

/**
 * @brief	change the mode of the e-puck to the given mode
 *
 * @param 	new_mode: new mode of the e_puck
 *
 */
void change_mode (mode_puck_t new_mode){
	mode=new_mode;
}


/**
 * @brief	returns the current mode
 *
 * @return 	current mode
 *
 */
mode_puck_t get_mode (void){
	return mode;
}


/*
 * @brief	checks if the robot is calibrating or not
 *
 * @return 	the state of 'calibrating' (true if the robot is
 * 			calibrating, false otherwise)
 */
bool get_calibrating(void){
	return calibrating;
}


/**
 * @brief	sets the changing_mode variable with the given value
 *
 * @param 	changing_value: value to set
 *
 */
void set_changing_mode(bool changing_value) {
	changing_mode=changing_value;
}


/**
 * @brief	sets the stop variable with the given value
 *
 * @param 	stop_value: value to set
 *
 */
void set_stop (bool stop_value){
	stop =stop_value;
}


/**
 * @brief	starts the thread responsible for detecting obstacles
 * 			while the robot is returning home
 */
void detect_collision_start(void){
	chThdCreateStatic(waThdCollision, sizeof(waThdCollision), NORMALPRIO, ThdCollision, NULL);
}


/**
 * @brief	sets the starting position of the robot and starts the thread responsible
 * 			for constantly tracking the position of the robot.
 */
void robot_position_start(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	chThdCreateStatic(waRobotPosition, sizeof(waRobotPosition), NORMALPRIO+1, RobotPosition, NULL);
}








