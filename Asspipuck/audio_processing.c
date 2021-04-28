#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <movements.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static uint8_t state [NB_STATES];


#define MIN_VALUE_THRESHOLD	50000

#define MIN_FREQ				130	//we don't analyze before this index to not use resources for nothing
#define FREQ_HALT				131	//2000Hz
#define FREQ_SOFT_CLEANING		144	//2200Hz
#define FREQ_DEEP_CLEANING		157	//2400Hz
#define FREQ_HOME				170	//2600Hz
#define MAX_FREQ				171	//we don't analyze after this index to not use resources for nothing

#define FREQ_HALT_L				(FREQ_HALT-1)
#define FREQ_HALT_H				(FREQ_HALT+1)
#define FREQ_SOFT_CLEANING_L	(FREQ_SOFT_CLEANING-1)
#define FREQ_SOFT_CLEANING_H	(FREQ_SOFT_CLEANING+1)
#define FREQ_DEEP_CLEANING_L	(FREQ_DEEP_CLEANING-1)
#define FREQ_DEEP_CLEANING_H	(FREQ_DEEP_CLEANING+1)
#define FREQ_HOME_L				(FREQ_HOME-1)
#define FREQ_HOME_H				(FREQ_HOME+1)

#define NO_INDEX		-1		//value given to the index when no index is chosen yet
#define TIMEOUT_VALUE 	100		//timeout value


/**
 * @brief	change the mode of the robot depending on the frequency of the
 * 			sound played.
 *
 * @param 	data: table containing the magnitude calculated using the
 * 			FFT transform
 */
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = NO_INDEX;

	static uint8_t timeout =0;

	if (timeout >TIMEOUT_VALUE){
		for (uint8_t i =0; i< NB_STATES; i++){
			state[i]=0;
		}
	}

	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(max_norm_index >= FREQ_HALT_L && max_norm_index <= FREQ_HALT_H){
		state[HALT]++;
	}

	else if(max_norm_index >= FREQ_SOFT_CLEANING_L && max_norm_index <= FREQ_SOFT_CLEANING_H){
		state[SOFT_CLEANING]++;
	}

	else if(max_norm_index >= FREQ_DEEP_CLEANING_L && max_norm_index <= FREQ_DEEP_CLEANING_H){
		state[DEEP_CLEANING]++;
	}

	else if(max_norm_index >= FREQ_HOME_L && max_norm_index <= FREQ_HOME_H){
		state[RETURN_HOME]++;
	}

	uint8_t max=state[0];
	uint8_t idx=0;
	for (uint8_t i =1; i< NB_STATES; i++){
		if (state[i]>max){
			max =state[i];
			idx=i;
		}
	}
	if (max> 5){
		for (uint8_t i =0; i< NB_STATES; i++){
			state[i]=0;
		}
		if (idx!=get_mode()){
			switch (idx){
			case 0:
				set_stop(true);
				set_changing_mode(true);
				change_mode(HALT);
				break;
			case 1:
				set_stop (true);
				set_changing_mode(true);
				change_mode(SOFT_CLEANING);
				break;
			case 2:
				set_stop (true);
				set_changing_mode(true);
				change_mode(DEEP_CLEANING);
				break;
			case 3:
				set_stop (true);
				set_changing_mode(true);
				change_mode(RETURN_HOME);
				break;
			}
		}
	}

}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part

		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;
		sound_remote(micLeft_output);
	}
}



/*
*	@brief 	wait for the sendToComputer_sem
*/
void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}


/*
*	@brief 	get the pointer of the corresponding buffer
*
*	@param	BUFFER_NAME_t: buffer name of the wanted pointer.
*	return	pointer of the buffer
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
