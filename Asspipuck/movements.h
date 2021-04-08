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
void rotate_rad(float angle, float speed);

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
float angle_reflection (float angle_colision);



void position_mode(float pos_r, float pos_l, float speed_r,  float speed_l);


#endif /* MOVEMENTS_H_ */
