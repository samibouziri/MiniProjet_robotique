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


void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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
	serial_start();
	spi_comm_start();
	usb_start();

	chThdSleepMilliseconds(2000);

	motors_init();
	proximity_start();
	calibrate_ir();
	mic_start(&processAudioData);
	robot_position_start();
	threads_start();
	VL53L0X_start();
	dcmi_start();
	po8030_start();
	process_image_start();

	while (1)
	{
		operating_mode();
	}
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
