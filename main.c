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
#include <camera/po8030.h>
#include <chprintf.h>
#include "jump.h"
#include "proximity_detection.h"
#include "regulator.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <chprintf.h>

#include <i2c_bus.h>

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

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // motors initialization
	motors_init();

	// proximity sensor initialization
	proximity_start();
	calibrate_ir();


	//start thread movement
	//start_speed();
	start_regulator();

	//start proximity detection
	start_proximity_detection();


    /* Infinite loop. */
    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
