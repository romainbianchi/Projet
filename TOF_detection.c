#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <selector.h>

#include "main.h"
#include "TOF_detection.h"
#include "regulator.h"
#include "gravity_detection.h"

static bool object_detected = false;
static bool floor_detected = false;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waTofDetection, 128);
static THD_FUNCTION(TofDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint16_t TOF_value = 0;

	systime_t time = 0;

	while(1){
		time = chVTGetSystemTime();

		TOF_value = VL53L0X_get_dist_mm();

		if(get_function_mode() == PARABOLA_FUNCTION_MODE && TOF_value < GOAL_FLOOR_DETECT){
			floor_detected = true;
		}

		if(get_function_mode() == NORMAL_FUNCTION_MODE){
			if(TOF_value < GOAL_TOF_VALUE){
				object_detected = true;
			}else{
				object_detected = false;
			}
		}

		//MODE CONDITIONS
		if(get_selector() ==  SELECT_START){
			if(object_detected && get_function_mode() == NORMAL_FUNCTION_MODE){
				set_function_mode(ROTATION_FUNCTION_MODE);
			}
			if(get_floor_detected() && get_function_mode() == PARABOLA_FUNCTION_MODE){
				set_function_mode(LANDING_FUNCTION_MODE);
			}
		}

		chThdSleepMilliseconds(3);
		//chThdSleepUntilWindowed(time, time + MS2ST(5));
	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_tof_detection(void){
	chThdCreateStatic(waTofDetection, sizeof(waTofDetection), NORMALPRIO, TofDetection, NULL);
}

bool get_object_detected(void){
	return object_detected;
}

bool get_floor_detected(void){
	return floor_detected;
}

void set_floor_detected(bool value){
	floor_detected = value;
}
