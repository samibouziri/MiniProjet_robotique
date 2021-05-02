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
#include <spi_comm.h>

#include <movements.h>
#include <ir_sensor.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/VL53L0X/Api/core/inc/vl53l0x_api.h>
#include <process_image.h>



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
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//starts the serial communication
	serial_start();

	//starts the spi communication
	spi_comm_start();

	//starts the USB communication
	usb_start();

	//wait for the epuck to be stable
	chThdSleepMilliseconds(2000);

	//initialize the motors
	motors_init();

	//start the proximity sensors
	proximity_start();

	//calibrate the proximity sensors
	calibrate_ir();

	//start the microphone
	mic_start(&processAudioData);

	//start the robot position thread
	robot_position_start();

	//start the detection of collision
	threads_start();

	//starts the camera
	dcmi_start();
	po8030_start();

	operating_mode();
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}

