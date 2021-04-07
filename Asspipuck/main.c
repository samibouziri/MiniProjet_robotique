#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>

#include <movements.h>
#include <ir_sensor.h>
#include <sensors/proximity.h>
#include <motors.h>

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

	//inits the motors
	motors_init();
	proximity_start();
	calibrate_ir();

	int16_t speed=2000;


	/* Infinite loop. */
	while (1) {

		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		if (colision_detected()){
			chprintf((BaseSequentialStream *)&SD3, "angle of reflection : %f \r\n",angle_reflection (angle_colision ()) );
			rotate_rad(angle_reflection (angle_colision ()),speed);
		}
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		//waits 10 ms
		chThdSleepMilliseconds(10);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
