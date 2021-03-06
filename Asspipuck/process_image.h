/*	Project: AspiPuck
 * 	process_image.h
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was modified by the authors. The original document was given in the
 * moodle of the course MICRO-315 under TP4_CamReg
 */

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/**
 * @brief	starts the thread responsible for the capturing of the
 * 			image and the thread responsible processing of the image
 */
void process_image_start(void);

/**
 * @brief	terminates the thread responsible for the capturing of the
 * 			image and the thread responsible processing of the image
 */
void process_image_stop(void);

/**
 * @brief	return the value of the variable detected
 * @return	a boolean which tells if the two lines were
 * 			detected or not
 */
bool get_detected(void);

#endif /* PROCESS_IMAGE_H */
