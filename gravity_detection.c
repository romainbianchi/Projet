#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <motors.h>
#include <selector.h>

#include "regulator.h"
#include "TOF_detection.h"
#include "gravity_detection.h"

#define	ANGLE_AT_NINETY			100000
#define ANGLE_PARABOLA_SUPP		4.2f
#define ANGLE_PARABOLA_INF		2.3f
#define ANGLE_LANDING_SUPP		0.15f
#define ANGLE_LANDING_INF		0.0f

static float angle_from_horizontal = 0; // in °
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

static THD_WORKING_AREA(waGravity, 512);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    static uint8_t count = 0;

	while(1){

    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	determine_angle(imu_values);

    	//MODE CONDITIONS
    	if(get_selector() ==  SELECT_START){

//    		//PIORITY SET
//    		if(get_function_mode() == ROTATION_FUNCTION_MODE || INV_ROTATION_FUNCTION_MODE){
//    			chThdSetPriority(NORMALPRIO+1);
//    		}else{
//    			chThdSetPriority(NORMALPRIO);
//    		}

    		//SPECIFIC ANGLES DETECTION
			if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP && quadrant == 1
					&& (get_function_mode() == ROTATION_FUNCTION_MODE || get_function_mode() == INV_ROTATION_FUNCTION_MODE)){
				stop_motors();
				set_function_mode(CONTROL_ANGLE_FUNCTION_MODE);
			}

			if(get_function_mode() == CONTROL_ANGLE_FUNCTION_MODE){
				if(count == 10){
					count = 0;
					determine_angle(imu_values);
					if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP && quadrant == 1){
						set_function_mode(PARABOLA_FUNCTION_MODE);
					}else if(angle_from_horizontal < ANGLE_PARABOLA_SUPP){
						set_function_mode(ROTATION_FUNCTION_MODE);
					}else{
						set_function_mode(INV_ROTATION_FUNCTION_MODE);
					}
				}else{
					count ++;
				}
			}

    		if(angle_from_horizontal > ANGLE_LANDING_INF && angle_from_horizontal < ANGLE_LANDING_SUPP && (quadrant == 1 || quadrant == 4)
    		   && get_function_mode() == LANDING_FUNCTION_MODE){
    			set_function_mode(NORMAL_FUNCTION_MODE);
    		}

    	}

    	chThdSleepMilliseconds(5);
	}
}

//------------------------------- EXTERNAL FUNCTIONS -------------------------------

void start_gravity(void){
	chThdCreateStatic(waGravity, sizeof(waGravity), NORMALPRIO+1, Gravity, NULL);
}
