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
void rotate_rad(float angle, uint16_t speed);

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
float get_translation (void);

/**
 * @brief	returns the angle of rotation of the epuck from the initalization point
 *
 * @return	the angle of rotation in rad
 */
int32_t get_rotation (void);

/**
 * @brief	converts a distance given in number of steps to a distance in cm
 *
 * @param 	nb_step 	distance in number of steps.
 * @return	distance in cm
 */
float step_to_cm (uint32_t nb_step);

float get_x();

float get_y();

int16_t get_angle();

void robot_position_start(void);

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
void position_mode(float pos_r, float pos_l, uint16_t speed_r,  uint16_t speed_l);


#endif /* MOVEMENTS_H_ */
