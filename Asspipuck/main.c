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

<<<<<<< HEAD
	/* Infinite loop. */
	while (1) {

		move_forward(0.5, 500);
		//waits 10 ms
		chThdSleepMilliseconds(1000);
=======
	robot_position_start();

	int16_t speed=500;
	chThdSleepMilliseconds(10000);


	/* Infinite loop. */
	while (1) {

		position_mode(10, 10, speed,  speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		rotate_rad(M_PI/2, speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		position_mode(10, 10, speed,  speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		position_mode(-10, -10, speed,  speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		rotate_rad(-M_PI/12, speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		rotate_rad(M_PI/12, speed);
		chprintf((BaseSequentialStream *)&SD3, "x = %f y= %f angle=%d\n\r",get_x(),get_y(),get_angle());
		//waits 3 sec
		chThdSleepMilliseconds(4000);
>>>>>>> origin/main
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
