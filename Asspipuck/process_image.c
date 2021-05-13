/*	Project: AspiPuck
 * 	process_image.c
 *
 *  Created on: 7 avr. 2021
 *     Authors: Sami Bouziri
 *      		Amine Tourki
 *
 * This document was modified by the authors. The original document was given in the
 * moodle of the course MICRO-315 under TP4_CamReg
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <process_image.h>

static thread_t *captImThd;
static thread_t *proImThd;
static bool detected =false;
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


#define IMAGE_BUFFER_SIZE		    640
#define NB_PIXEL_FOR_STABILIZATION	10
#define MIN_DECREASE				15
#define MIN_WIDTH					5
#define PROPORTIONALITY_FACTOR		2


/**
 * @brief	detect two red lines with the same width
 * 			and a white space between with also their width
 *
 * @param	image	image of the two lines (1 by 640 array of uint8_t)
 * @param	mean 	the mean value of all the pixels in the image
 */
bool line_detection(const uint8_t* image,uint32_t mean);

//----------Private Function----------

static THD_WORKING_AREA(waCaptureImage, 512);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while(chThdShouldTerminateX() == false){
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
	}
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	//uint8_t send=0;

	while(chThdShouldTerminateX() == false){

		//waits until an image has been captured
		chBSemWait(&image_ready_sem);

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint32_t mean=0;
		for (uint16_t i=0; i<IMAGE_BUFFER_SIZE*2;i=i+2)
		{
			//extract the green value
			image[i>>1] = ((((img_buff_ptr[i]&7)<<3)|((img_buff_ptr[i+1]&224)>>5))<<2) ;
			// sum all the intensities of the image to calculate the mean
			mean+= image[i/2];
		}
		//calculate the mean intesity of the image
		mean/=IMAGE_BUFFER_SIZE;

		// detect two red lines with the same width seperated by their width
		detected =line_detection(image,mean);
	}

}

/**
 * @brief	detect two red lines with the same width
 * 			and a white space between with also their width
 *
 * @param	image	image of the two lines (1 by 640 array of uint8_t)
 * @param	mean 	the mean value of all the pixels in the image
 */
bool line_detection(const uint8_t* image,uint32_t mean)
{

	bool  downthr=false;
	uint16_t widthline=1, centerline=0;
	uint16_t linestart=0;

	//detection of the first line
	for (uint16_t i=0; i<IMAGE_BUFFER_SIZE-NB_PIXEL_FOR_STABILIZATION ; i++ )
	{
		//detection of a fall of the first line
		if (	image[i]>=mean &&
				image[i+NB_PIXEL_FOR_STABILIZATION]<mean-MIN_DECREASE  &&
				!downthr)
		{
			downthr=true;
			linestart=i;
		}
		//detection of a rise of the first line
		if (	image[i]<=mean-MIN_DECREASE &&
				image[i+NB_PIXEL_FOR_STABILIZATION]>mean &&
				downthr && i+1-linestart>widthline)
		{
			widthline= i+1-linestart;
			centerline=widthline/2+linestart;
			downthr=false;
		}

	}
	//verification that the first line was well detected
	if (widthline>MIN_WIDTH){
		//detection of the second line on the left of the first line
		downthr=false;
		uint16_t widthline1=1, centerline1=0;
		uint16_t linestart1=0;
		for (uint16_t i=0; i<centerline-widthline/2-NB_PIXEL_FOR_STABILIZATION ; i++ )
		{
			//detection of a fall of the second line
			if (	image[i]>=mean &&
					image[i+NB_PIXEL_FOR_STABILIZATION]<mean-MIN_DECREASE &&
					!downthr )
			{
				downthr=true;
				linestart1=i;
			}
			//detection of a rise of the second line
			if (	image[i]<=mean-MIN_DECREASE &&
					image[i+NB_PIXEL_FOR_STABILIZATION]>mean &&
					downthr && i+1-linestart1>widthline1)
			{
				widthline1= i+1-linestart1;
				centerline1=widthline1/2+linestart1;
				downthr=false;
			}
		}

		//detection of the second line on the right of the first line
		downthr=false;
		for (uint16_t i=centerline+widthline/2+1; i<IMAGE_BUFFER_SIZE-NB_PIXEL_FOR_STABILIZATION ; i++ )
		{
			//detection of a fall of the second line
			if (	image[i]>=mean &&
					image[i+NB_PIXEL_FOR_STABILIZATION]<mean-MIN_DECREASE &&
					!downthr )
			{
				downthr=true;
				linestart1=i;
			}
			//detection of a rise of the second line
			if (	image[i]<=mean-MIN_DECREASE &&
					image[i+NB_PIXEL_FOR_STABILIZATION]>mean &&
					downthr && i+1-linestart1>widthline1)
			{
				widthline1= i+1-linestart1;
				centerline1=widthline1/2+linestart1;
				downthr=false;
			}
		}
		//verification that the second line is valid and the two lines and the white space between them
		//have nearly the same width (taking into account the distortion of the image )
		if ( 	widthline1>MIN_WIDTH &&
				(widthline1<=widthline*PROPORTIONALITY_FACTOR && widthline1>=widthline/PROPORTIONALITY_FACTOR) &&
				((abs(centerline-centerline1)-widthline1/2-widthline/2)<=widthline*PROPORTIONALITY_FACTOR &&
						( abs(centerline-centerline1)-widthline1/2-widthline/2)>=widthline/PROPORTIONALITY_FACTOR))
		{
			return true ;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

//----------Public Function----------

/**
 * @brief	return the value of the variable detected
 * @return	a boolean which tells if the two lines were
 * 			detected or not
 */
bool get_detected(void){
	return detected;
}

/**
 * @brief	starts the thread responsible for the capturing of the
 * 			image and the thread responsible processing of the image
 */
void process_image_start(void){
	captImThd=chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	proImThd=chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);

}

/**
 * @brief	terminates the thread responsible for the capturing of the
 * 			image and the thread responsible processing of the image
 */
void process_image_stop(void) {
	chThdTerminate(proImThd);
	chThdWait(proImThd);
	proImThd = NULL;
	chThdTerminate(captImThd);
	chThdWait(captImThd);
	captImThd = NULL;
}
