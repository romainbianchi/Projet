#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <selector.h>
#include <chprintf.h>

static float angle_from_horizontal = 0.0; // raaport entre x et y, =1 pour 45° et
static uint8_t quadrant = 0; // dans quel quadrant pointe le robot


//------------------------------- INTERNAL FUNCTIONS --------------------------------

void determine_angle(imu_msg_t imu_values){
	float *accel = imu_values.acceleration;
	if(accel[X_AXIS] < 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 1;
		}else{
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 4;
		}
	}else if(accel[X_AXIS] > 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 2;
		}else{
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 3;
		}
	}else if(accel[X_AXIS] == 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = ANGLE_AT_NINETY;
			quadrant = 1;
		}else{
			angle_from_horizontal = -ANGLE_AT_NINETY;
			quadrant = 4;
		}
	}
}

static THD_WORKING_AREA(waGravity, 256);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

	while(1){
		if(get_selector() == 8){
			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    		determine_angle(imu_values);
    	}

    	chThdSleepMilliseconds(10);
	}
}

//------------------------------- EXTERNAL FUNCTIONS -------------------------------

void start_gravity(void){
	chThdCreateStatic(waGravity, sizeof(waGravity), NORMALPRIO, Gravity, NULL);
}

float get_angle(void){
	return angle_from_horizontal;
}

uint8_t get_quadrant(void){
	return quadrant;
}
