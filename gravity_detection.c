#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>

static float angle_from_horizontal = 0; // in °


//------------------------------- INTERNAL FUNCTIONS --------------------------------

float determine_angle(imu_msg_t imu_values){
	float *accel = imu_values.acceleration;
	return atan2(accel[Y_AXIS], -accel[X_AXIS]);
}

static THD_WORKING_AREA(waGravity, 256);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

	while(1){
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	angle_from_horizontal = determine_angle(imu_values)*180/PI;
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
