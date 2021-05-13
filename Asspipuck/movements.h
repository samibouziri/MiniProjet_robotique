/*	Project: AspiPuck
 * 	movements.h
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was created by the authors.
 */

#ifndef MOVEMENTS_H_
#define MOVEMENTS_H_


typedef enum{
  HALT,
  SOFT_CLEANING,
  DEEP_CLEANING,
  RETURN_HOME,
  CHARGING
}mode_puck_t;



/**
 * @brief	sets the starting position of the robot and starts the thread responsible
 * 			for constantly tracking the position of the robot.
 */
void robot_position_start(void);


/**
 * @brief	starts the thread responsible for detecting obstacles
 * 			while the robot is returning home
 */
void detect_collision_start(void);


/**
 * @brief	change the mode of the e-puck to the given mode
 *
 * @param 	new_mode: new mode of the e_puck
 *
 */
void change_mode (mode_puck_t new_mode);


/**
 * @brief	operate the e-puck in the mode corresponding to the state
 * 			of the variable mode
 */
void operating_mode(void);


/**
 * @brief	sets the stop variable with the given value
 *
 * @param 	stop_value: value to set
 *
 */
void set_stop (bool stop_value);


/**
 * @brief	sets the changing_mode variable with the given value
 *
 * @param 	changing_value: value to set
 *
 */
void set_changing_mode(bool changing_value);


/**
 * @brief	returns the current mode
 *
 * @return 	current mode
 *
 */
mode_puck_t get_mode (void);


/*
 * @brief	checks if the robot is calibrating or not
 *
 * @return 	the state of 'calibrating' (true if the robot is
 * 			calibrating, false otherwise)
 */
bool get_calibrating(void);


#endif /* MOVEMENTS_H_ */
