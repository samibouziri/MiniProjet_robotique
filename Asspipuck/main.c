#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <math.h>
#include <chprintf.h>

#include <movements.h>
#include <ir_sensor.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
			115200,
			0,
			0,
			0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{

	halInit();
	chSysInit();
	mpu_init();
	chThdSleepMilliseconds(2000);
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	//starts the serial communication
	serial_start();


	chThdSleepMilliseconds(2000);
	//inits the motors
	motors_init();
	proximity_start();
	calibrate_ir();
	//robot_position_start();
	//temp tab used to store values in complex_float format
	//needed bx doFFT_c
	/*static complex_float temp_tab[FFT_SIZE];
	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	static float send_tab[FFT_SIZE];
	mic_start(&processAudioData);


	int16_t speed =500;
	chThdSleepMilliseconds(4000);
	// Infinite loop.
	while (1) {
		wait_send_to_computer();
		arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
		SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);

		chThdSleepMilliseconds(4000);
	}*/
	search_obstacle();
	while (1){
		turn_around_clockwise_speed();
		//chprintf((BaseSequentialStream *)&SD3, "prox(2)= %d prox(3)= %d \r\n",get_calibrated_prox(1),get_calibrated_prox(2));
		chThdSleepMilliseconds(15);
	}

}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
