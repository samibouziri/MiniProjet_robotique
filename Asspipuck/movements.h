/*
 * movements.h
 *
 *  Created on: 7 avr. 2021
 *      Author: sami bouziri
 */

#ifndef MOVEMENTS_H_
#define MOVEMENTS_H_

/**
 * @brief	rotates the e_puck with a certain angle and speed
 *
 * @param	angle	angle of rotation (in rad)
 * @param	speed	speed of rotation (in step/s)
 */
void rotate_rad(float angle, int16_t speed);

/**
 * @brief	moves the robot forward with a certain speed
 *
 * @param 	distance (in cm) : distance to travel
 * @param 	speed ((in step/s): speed of travel

 */

void move_forward(float distance, uint16_t speed );

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */

float angle_reflection (float angle_colision);

/**
 * @brief	returns the translation of the epuck from the initalization point
 *
 * @return	the translation in cm
 */
float get_translation (int32_t last_right_motor_pos,int32_t last_left_motor_pos);

/**
 * @brief	returns the angle of rotation of the epuck from the initalization point
 *
 * @return	the angle of rotation in rad
 */
int32_t get_rotation (int32_t last_right_motor_pos,int32_t last_left_motor_pos);

/**
 * @brief	converts a distance given in number of steps to a distance in cm
 *
 * @param 	nb_step 	distance in number of steps.
 * @return	distance in cm
 */
float step_to_cm (int32_t nb_step);

float get_x(void);



float get_y(void);


int16_t get_angle(void);

void robot_position_start(void);

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
void position_mode(float pos_r, float pos_l, int16_t speed_r,  int16_t speed_l);



/**
 * @brief	turns in circle around an obstacle clockwise
 *
 */



void turn_around_anticlockwise_speed(void);

void turn_around_clockwise_speed(void);

bool search_obstacle (void);


#endif /* MOVEMENTS_H_ */
