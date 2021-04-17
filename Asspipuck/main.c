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


	chThdSleepMilliseconds(2000);
	//inits the motors
	motors_init();
	proximity_start();
	calibrate_ir();
	bool rotate=search_obstacle();

	//chThdSleepMilliseconds(1000);


//	chprintf((BaseSequentialStream *)&SD3, "sensor1 = %d cm \r\n",sensor_close_obstacle(SENSOR_1));
//	chprintf((BaseSequentialStream *)&SD3, "sensor3 = %d cm \r\n",sensor_detection(SENSOR_3));

	//turn_around_clockwise_speed();
	//chThdSleepMilliseconds(10);
	if (rotate)
	{
		while (1)
	{
		turn_around_clockwise_speed();
		chThdSleepMilliseconds(15);
	}
	}
	else
	{
		while (1)
		{
			turn_around_anticlockwise_speed();
			chThdSleepMilliseconds(15);
		}
	}
	//turn_around_clockwise_speed();
	//change_state ();
	//state_move ();


}




#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
