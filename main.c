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
#include <camera/po8030.h>
#include <chprintf.h>
#include "jump.h"
#include "parabole.h"
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <chprintf.h>
#include "astolfix.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

#define NB_SAMPLES_OFFSET     200


//static void serial_start(void){
//	static SerialConfig ser_cfg = {
//			115200,
//			0,
//			0,
//			0,
//	};
//
//	sdStart(&SD3, &ser_cfg);
//}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
	imu_start();
	i2c_start();
//	serial_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
//    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
//    imu_msg_t imu_values;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
//    calibrate_gyro();
//    calibrate_acc();

    // motors initialization
	motors_init();


//	chprintf((BaseSequentialStream *)&SD3, "-----------------%f\r\n", roc);


//	for(float t=0; t<10; t+=0.1){

//		roc = calculate_roc(t, a, b, d);
//		v = calculate_norm_speed(t);
//		chprintf((BaseSequentialStream *)&SD3, "vitesse = %f\r\n", v);
//		chprintf((BaseSequentialStream *)&SD3, "roc = %f\r\n", roc);


//		left_motor_set_speed(speed_conversion_cm_to_step(calculate_outer_speed(roc, v)));
//		right_motor_set_speed(speed_conversion_cm_to_step(calculate_inner_speed(roc, v)));
//	}
//	left_motor_set_speed(0);
//	right_motor_set_speed(0);

	start_parabola();
//	start_astolfi();
    // Infinite loop
    while (1) {
//    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
//    	show_gravity(imu_values);
    	palTogglePad(GPIOD, GPIOD_LED1);

//    	chThdSleepMilliseconds(50);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


//void show_gravity(imu_msg_t imu_values){
//
//    //we create variables for the led in order to turn them off at each loop and to
//    //select which one to turn on
//    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
//    //threshold value to not use the leds when the robot is too horizontal
//    float threshold = 0.2;
//    //create a pointer to the array for shorter name
//    float *accel = imu_values.acceleration;
//
//     chSysLock();
//
//     //we find which led of each axis should be turned on
//     if(accel[X_AXIS] > threshold)
//         led7 = 1;
//     else if(accel[X_AXIS] < -threshold)
//         led3 = 1;
//
//     if(accel[Y_AXIS] > threshold)
//         led5 = 1;
//     else if(accel[Y_AXIS] < -threshold)
//         led1 = 1;
//
//     //if two leds are turned on, turn off the one with the smaller
//     //accelerometer value
//     if(led1 && led3){
//         if(accel[Y_AXIS] < accel[X_AXIS])
//             led3 = 0;
//         else
//             led1 = 0;
//     }else if(led3 && led5){
//         if(accel[X_AXIS] < -accel[Y_AXIS])
//             led5 = 0;
//         else
//             led3 = 0;
//     }else if(led5 && led7){
//         if(accel[Y_AXIS] > accel[X_AXIS])
//             led7 = 0;
//         else
//             led5 = 0;
//     }else if(led7 && led1){
//         if(accel[X_AXIS] > -accel[Y_AXIS])
//             led1 = 0;
//         else
//             led7 = 0;
//     }
//     chSysUnlock();
//
//    //we invert the values because a led is turned on if the signal is low
//    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);
//
//}

