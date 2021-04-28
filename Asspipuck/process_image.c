#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <process_image.h>

bool line_detection(const uint8_t* image, uint16_t* width, uint32_t somme);
static float distance_cm = 0;
static uint16_t center=0;

#define d_lens 0.21168f
#define pixel_size 2.8e-4f
#define line_width 2.0f
#define IMAGE_BUFFER_SIZE		640

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static bool detected =false;

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	//systime_t time;


	while(1){
	        //starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
	//		 chThdSleepMilliseconds(12);

	}
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t send=0;
	uint16_t width=0;

	while(1){
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();
		uint32_t somme=0;
		for (uint16_t i=0; i<IMAGE_BUFFER_SIZE*2;i=i+2)
		{

			image[i>>1] = ((((img_buff_ptr[i]&7)<<3)|((img_buff_ptr[i+1]&224)>>5))<<2) ;
			somme+= image[i/2];
		}
		somme/=IMAGE_BUFFER_SIZE;

		if (send){
			SendUint8ToComputer(image,IMAGE_BUFFER_SIZE);
		}
		send= (send+1)&1;

		// detect black line
		detected =line_detection(image,&width,somme);

		distance_cm=(line_width*d_lens)/(pixel_size*width);
	}


}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_center_line(void){
	return center;
}

bool get_detected(void){
	return detected;
}



void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

bool line_detection(const uint8_t* image, uint16_t* width, uint32_t somme)
{

	bool  downthr=false;
	uint16_t widthline=1, centerline=0;
	uint16_t linestart=0;


	for (uint16_t i=0; i<IMAGE_BUFFER_SIZE-10 ; i++ )
	{
		if (image[i]>=somme && image[i+10]<somme-20  && !downthr)
		{
			downthr=true;
			linestart=i;
		}
		if (image[i]<=somme-20 && image[i+10]>somme && downthr && i+1-linestart>widthline)
		{

			widthline= i+1-linestart;
			centerline=widthline/2+linestart;
			downthr=false;

		}

	}
	if (widthline>5){
		downthr=false;
		uint16_t widthline1=1, centerline1=0;
		uint16_t linestart1=0;

		for (uint16_t i=0; i<centerline-widthline/2-10 ; i++ )
		{
			if (image[i]>=somme && image[i+10]<somme-15 &&!downthr )
			{
				downthr=true;
				linestart1=i;
			}
			if (image[i]<=somme-15 && image[i+10]>somme && downthr && i+1-linestart1>widthline1)
			{

				widthline1= i+1-linestart1;
				centerline1=widthline1/2+linestart1;
				downthr=false;
			}
		}
		downthr=false;
		for (uint16_t i=centerline+widthline/2+1; i<IMAGE_BUFFER_SIZE-10 ; i++ )
		{
			if (image[i]>=somme && image[i+10]<somme-15 &&!downthr )
			{
				downthr=true;
				linestart1=i;
			}
			if (image[i]<=somme-15 && image[i+10]>somme && downthr && i+1-linestart1>widthline1)
			{

				widthline1= i+1-linestart1;
				centerline1=widthline1/2+linestart1;
				downthr=false;
			}
		}
		if ( 	widthline1>5 &&
				((float)widthline1<=(float)widthline*1.6 && (float)widthline1>=(float)widthline/1.6) &&
				(float)((abs(centerline-centerline1)-widthline1/2-widthline/2)<=(float)widthline*1.6 &&
						(float)( abs(centerline-centerline1)-widthline1/2-widthline/2)>=(float)widthline/1.6)){
			return true ;
		}else {
			return false;
		}
	}
	else{
		return false;
	}

	*width=widthline;
	center=centerline;
}







