#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include "memory_protection.h"
#include <usbcfg.h>
#include <leds.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <selector.h>

#include "main.h"
#include "proximity_detection.h"
#include "TOF_detection.h"
#include "regulator.h"
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

static uint8_t function_mode = NORMAL_FUNCTION_MODE; //variable used to set the different modes

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    serial_start();
    imu_start();


    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    /* motors initialization */
	motors_init();

	/* proximity sensor initialization */
	proximity_start();
	calibrate_ir();

	/* TOF start */
	VL53L0X_start();
	chThdSleepMilliseconds(500);


	/* acc calibration */
    chThdSleepMilliseconds(1000);
    calibrate_acc();

	/* start proximity detection */
	start_proximity_detection();

	/* start TOF detection */
	start_tof_detection();

	/* start gravity detection */
	start_gravity();

	/* start thread movement */
	start_regulator();

	/* end of initialization signal */
	set_body_led(1);
	chThdSleepMilliseconds(200);
	set_body_led(0);

    /* Infinite loop. */
    while (1) {

    }
}

/* return the actual mode */
uint8_t get_function_mode(void){
	return function_mode;
}

/* set the mode */
void set_function_mode(uint8_t mode){
	function_mode = mode;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
