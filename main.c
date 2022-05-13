#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <chprintf.h>

#include "proximity_detection.h"
#include "TOF_detection.h"
#include "regulator.h"
#include "parabole.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <chprintf.h>

#include <i2c_bus.h>
#include "parabole.h"

#include <sensors/imu.h>
#include "gravity_detection.h"

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

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    serial_start();
    i2c_start();
    imu_start();


    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);



    // motors initialization
	motors_init();

	// acc calibration
    chThdSleepMilliseconds(2000);
    calibrate_acc();
    // end of initialization
    set_body_led(1);
    chThdSleepMilliseconds(1000);
    set_body_led(0);




	// proximity sensor initialization
	proximity_start();
	calibrate_ir();

	// TOF start
	VL53L0X_start();
	chThdSleepMilliseconds(500);

	//start proximity detection
	start_proximity_detection();

	// start TOF detection
	start_tof_detection();

	// start gravity
	start_gravity();

	//start thread movement
	start_regulator();

    //start_parabola();

    /* Infinite loop. */
    while (1) {
//    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
//    	show_gravity(imu_values);
//    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
