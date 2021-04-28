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
#include <leds.h>

#define NSTEP_ONE_TURN      1000	//number of steps needed to do a full turn of the wheel
#define WHEEL_DISTANCE      5.35f	//cm (the distance between the two wheels)
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE) // perimeter of the circle drawn by the wheels
#define WHEEL_PERIMETER     13.f
#define TURN_STEP			((PERIMETER_EPUCK/WHEEL_PERIMETER)*NSTEP_ONE_TURN) //nb steps for one full turn
#define HALT_SPEED			0
#define	SPEED				600
#define CLOSE_THR			100
#define EPSILON 			0.1f
#define MINIMAL_ANGLE_EXIT	M_PI/4
#define MAXIMAL_ANGLE_EXIT	3*M_PI/4

#define COLLISION_SLEEP		10		//sleeping time during the detection of a collision
#define GO_AND_AVOID_SLEEP	15		//sleeping time for the rotation around the obstacle
#define SOFT_CLEANING_SLEEP 100		//sleeping time in the soft cleaning mode to avoid detecting the same collision twice

#define BLUE	0,0,255
#define RED		255,0,0
#define GREEN	0,255,0
#define	WHITE	255,255,255
#define ORANGE	255,165,0

#define HOME_POS	0,0

static float x=0;
static float y=0;
static float angle=0;

static bool stop=false;
static bool arrived=false;
static bool changing_mode=false;
static mode_puck_t mode =HALT;

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
		last_right_motor_pos=right_motor_get_pos();
		last_left_motor_pos=left_motor_get_pos();
		x+=amplitude*cosf(angle);
		y+=amplitude*sinf(angle);
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}
/**
 * @brief	rotate the e-puck to the right or to the left depending on the angle
 *			of collision to get it parallel with the obstacle
 *
 * @return	true if rotation done to the left, false if rotation done to the right
 */
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

/**
 * @brief	checks if two vectors are collinear
 *
 * @param 	x1,y1	coordinates of the first vector (in cm)
 * @param	x2,y2	coordinates of the second vector (in cm)
 * @return	true if collinear with a threshold EPSILON, false otherwise
 */
bool collinear(float x1,float x2,float y1,float y2){
	if(abs(x1*y2-x2*y1)/sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))<EPSILON)
		return true;
	else
		return false;
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
	if (arrived)
	{
		right_motor_set_speed(HALT_SPEED);
		left_motor_set_speed(HALT_SPEED);
		return;
	}
	do
	{
		if (changing_mode){
			right_motor_set_speed(HALT_SPEED);
			left_motor_set_speed(HALT_SPEED);
			return;
		}
		chBSemSignal(&detect_obstacle_sem);
		go_to_xy (xg,yg,SPEED);
		if (changing_mode)
		{
			right_motor_set_speed(HALT_SPEED);
			left_motor_set_speed(HALT_SPEED);
			return;
		}
		stop=false;
		if(!arrived && !stop){
			float alpha=0;
			if (allign_to_avoid ()){
				do{
					if (changing_mode){
						right_motor_set_speed(HALT_SPEED);
						left_motor_set_speed(HALT_SPEED);
						return;
					}
					turn_around_clockwise_speed();
					chThdSleepMilliseconds(GO_AND_AVOID_SLEEP);
					alpha =atan2f((yg-y),(xg-x))-angle;
						if (alpha>M_PI)
							alpha -=2*M_PI;
						if (alpha<=-M_PI)
							alpha +=2*M_PI;
				}while (!(alpha>MINIMAL_ANGLE_EXIT && alpha< MAXIMAL_ANGLE_EXIT) && !stop);
			}
			else{
				do{
					if (changing_mode){
						right_motor_set_speed(HALT_SPEED);
						left_motor_set_speed(HALT_SPEED);
						return;
					}
					turn_around_anticlockwise_speed();
					chThdSleepMilliseconds(GO_AND_AVOID_SLEEP);
					alpha =atan2f((yg-y),(xg-x))-angle;
					if (alpha>M_PI)
						alpha -=2*M_PI;
					if (alpha<=-M_PI)
						alpha +=2*M_PI;
				}while (!(alpha>-MAXIMAL_ANGLE_EXIT && alpha<-MINIMAL_ANGLE_EXIT) && !stop);
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
		arrived =false;
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
				stop_r=true;
				right_motor_set_speed(HALT_SPEED);
			}
			if (abs(left_motor_get_pos()-left_pos)>=abs(pos_l*(NSTEP_ONE_TURN/WHEEL_PERIMETER)))
			{
				stop_l=true;
				left_motor_set_speed(HALT_SPEED);
			}
			if (stop_r && stop_l) {
				if (mode==RETURN_HOME)
				{
					arrived =true;
				}
				break;
			}
		}
	}
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
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

/**
 * @brief	returns the x coordinate of the robot
 *
 *
 * @return	coordinate x in cm
 */
float get_x() {
	return x;
}

/**
 * @brief	returns the y coordinate of the robot
 *
 * @return	coordinate y in cm
 */
float get_y() {
	return y;
}

/**
 * @brief	returns the angle of rotation of the robot compared to
 * 			the starting position
 *
 * @return	angle of rotation in degrees.
 */
float get_angle() {
	return angle*180/M_PI;
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

/**
 * @brief	go to the designed coordinates with a certain speed
 * 			starting from the current position
 *
 * @param	abscisse (in cm) : abscissa of the goal position
 * @param	ordonnee (in cm) : ordinate of the goal position
 * @param	speed (in step/s): speed of travel
 */
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
 * @brief	turns in circles clockwise around an obstacle
 *
*/
void turn_around_clockwise_speed(void){
	if (!sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		right_motor_set_speed(400);
		left_motor_set_speed(1000);
		return;
	}

	else if (sensor_close_obstacle(SENSOR_3,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR+40) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		if (sensor_close_obstacle(SENSOR_3,3*CLOSE_THR))
		{
			right_motor_set_speed(800);
			left_motor_set_speed(-800);
			return;
		}
		right_motor_set_speed(600);
		left_motor_set_speed(600);
		return;
	}
	else
	{
		left_motor_set_speed(-800);
		right_motor_set_speed(800);
	}
}

/**
 * @brief	goes forward while no obstacle is detected, then align the robot
 * 			if in front of an obstacle
 *
 * @return	true if the robot aligned itself to the left and false if the robot
 * 			aligned itself to the right.
*/
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
		return true;
	}
	else
	{
		rotate_rad(-M_PI/2+angle_colision(),600);
		return false;
	}
}


/**
 * @brief	goes forward while no obstacle is detected wnad while the robot is in
 * 			turn_arounf mode, then align the robot if in front of an obstacle
 *
 * @return	true if the robot aligned itself to the left and false if the robot
 * 			aligned itself to the right.
*/
bool search_obstacle_turn (void)
{
	while (!colision_detected() && mode==DEEP_CLEANING )
	{
		right_motor_set_speed(SPEED);
		left_motor_set_speed(SPEED);
	}
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


/**
 * @brief	turns in circles anticlockwise around an obstacle
 *
*/
void turn_around_anticlockwise_speed(void){
	if (!sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR)	)
	{
		right_motor_set_speed(1000);
		left_motor_set_speed(400);
		return;
	}

	else if (sensor_close_obstacle(SENSOR_6,CLOSE_THR) &&
			!sensor_close_obstacle(SENSOR_2,CLOSE_THR) &&
			!(sensor_close_obstacle(SENSOR_1,CLOSE_THR)||sensor_close_obstacle(SENSOR_8,CLOSE_THR))	 &&
			!sensor_close_obstacle(SENSOR_7,CLOSE_THR+40))
	{
		if (sensor_close_obstacle(SENSOR_3,3*CLOSE_THR))
		{
			right_motor_set_speed(-800);
			left_motor_set_speed(800);
			return;
		}
		right_motor_set_speed(600);
		left_motor_set_speed(600);
		return;
	}
	else
	{
		left_motor_set_speed(800);
		right_motor_set_speed(-800);
	}
}
static THD_WORKING_AREA(waThdCollision, 1024);
static THD_FUNCTION(ThdCollision, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (true)
	{

		chBSemWait(&detect_obstacle_sem);
		while (!colision_detected() && mode==RETURN_HOME)
		{
			chThdSleepMilliseconds(10);
		}
		if (mode==RETURN_HOME) {
			stop=true;
		}
	}
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
 * @brief	sets the changing_mode variable with the given value
 *
 * @param 	changing_value: value to set
 *
*/
void set_changing_mode(bool changing_value) {
	changing_mode=changing_value;
}


/**
 * @brief	starts the thread responsible for detecting obstacles
 * 			while the robot is in go_home mode
*/
void threads_start(void){
	chThdCreateStatic(waThdCollision, sizeof(waThdCollision), NORMALPRIO, ThdCollision, NULL);
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
		while (step && mode==SOFT_CLEANING)
		{
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepMilliseconds(SOFT_CLEANING_SLEEP);
			step=false;
		}
		while (mode==SOFT_CLEANING && !colision_detected())
		{
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
		}
		if (mode==SOFT_CLEANING && colision_detected())
		{
			rotate_rad(angle_reflection (angle_colision()),SPEED);
			step=true;
		}
		if (mode!=SOFT_CLEANING)
		{
			right_motor_set_speed(HALT_SPEED);
			left_motor_set_speed(HALT_SPEED);
			return;
		}
	}
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
		right_motor_set_speed(HALT_SPEED);
		left_motor_set_speed(HALT_SPEED);
	}
}


/**
 * @brief	operates the e-puck in deeo cleaning mode: in this mode, the
 * 			e-puck goes forward while no obstacle is detected then starts
 * 			circling around the first detected obstacle in a clockwise
 * 			or anticlockwise direction depending on the angle of collision
*/
void deep_cleaning(void)
{
	bool turn_clockwise=search_obstacle_turn();
	while (mode==DEEP_CLEANING)
	{
		if (turn_clockwise && mode==DEEP_CLEANING)
		{
			turn_around_clockwise_speed();
		}
		else if (!turn_clockwise && mode==DEEP_CLEANING)
		{
			turn_around_anticlockwise_speed();
		}
	}
	right_motor_set_speed(HALT_SPEED);
	left_motor_set_speed(HALT_SPEED);
	return;
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
			clear_leds();
			break;
		}
	}
}











