/*
 * movements.h
 *
 *  Created on: 7 avr. 2021
 *      Author: sami bouziri
 */

#ifndef MOVEMENTS_H_
#define MOVEMENTS_H_


typedef enum{
  HALT,
  SOFT_CLEANING,
  DEEP_CLEANING,
  RETURN_HOME,
}mode_puck_t;

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
 * @param 	speed (in step/s): speed of travel

 */

void move_forward(float distance, int16_t speed );

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
float get_rotation (int32_t last_right_motor_pos,int32_t last_left_motor_pos);

/**
 * @brief	converts a distance given in number of steps to a distance in cm
 *
 * @param 	nb_step 	distance in number of steps.
 * @return	distance in cm
 */
float step_to_cm (int32_t nb_step);

float get_x(void);



float get_y(void);

float get_angle(void);

void robot_position_start(void);
void go_to_xy (float abscisse, float ordonnee, int16_t speed);

/**
 * @brief	for a given angle of incidence of a collision with a
 * 			surface it returns the angle with which the e puck
 * 			should turn in order to be reflected
 *
 * @param 	angle_colision 	angle of incidence of the epuck(in rad)
 * @return	the angle with which the epuck must turn (in rad)
 */
void position_mode(float pos_r, float pos_l, int16_t speed_r,  int16_t speed_l);

void turn_around_clockwise_speed(void);

bool search_wall (void);


/**
 * @brief	turns in circle around an obstacle clockwise
 *
 */
void go_and_avoid(float xg, float yg);

void turn_around_anticlockwise_speed(void);

void turn_around_clockwise_speed(void);

void threads_start(void);

void change_mode (mode_puck_t new_mode);

void test_mode(void); ////////// à supprimer

void ricochet_mode(void);

void operating_mode(void);

void set_stop (bool stop_value);

void set_changing_mode(bool changing_value);

mode_puck_t get_mode (void);

void move_forward_speed( int16_t speed );

/**
 * @brief turn until it recognize 2 lines with the same width and
 *      seperated by the width of one of them
 */
void turn_patern_recognition(void);

/**
 * @brief makes the e-puck go forward until it reaches a wall and
 *      then place itself in front of it by a distance of 15 cm
 *      and in the middle of it (assuming that there are walls
 *      on its right and left)
 */
void calibration (void);

#endif /* MOVEMENTS_H_ */
