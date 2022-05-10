#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>

static float angle_from_horizontal = 0; //in °


//------------------------------- INTERNAL FUNCTIONS --------------------------------


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
////     chSysLock();
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
////     chSysUnlock();
//
//    //we invert the values because a led is turned on if the signal is low
//    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
//    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);
//
//}

float determine_angle(imu_msg_t imu_values){
	float *accel = imu_values.acceleration;
	return atan2(accel[Y_AXIS], -accel[X_AXIS]);
}

void set_angle(float angle){
	angle_from_horizontal = angle*180/PI;
}

static THD_WORKING_AREA(waGravity, 256);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;


	while(1){
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	set_angle(determine_angle(imu_values));
    	volatile float debug = angle_from_horizontal;
    	chThdSleepMilliseconds(200);
	}
}

//------------------------------- EXTERNAL FUNCTIONS -------------------------------

void start_gravity(void){
	chThdCreateStatic(waGravity, sizeof(waGravity), NORMALPRIO, Gravity, NULL);
}

float get_angle(void){
	return angle_from_horizontal;
}
