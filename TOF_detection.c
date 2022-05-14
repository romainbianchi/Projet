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

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waTofDetection, 256);
static THD_FUNCTION(TofDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint16_t TOF_value = 0;

	while(1){

		TOF_value = VL53L0X_get_dist_mm();

//		//PRIORITY SET
//		if(get_function_mode() == PARABOLA_FUNCTION_MODE || get_function_mode() == NORMAL_FUNCTION_MODE){
//			chThdSetPriority(NORMALPRIO+1);
//		}else{
//			chThdSetPriority(NORMALPRIO);
//		}

		//OBJECT DETECTION
		if(get_function_mode() == NORMAL_FUNCTION_MODE){
			if(TOF_value < GOAL_OBJECT_VALUE){
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
		}

		chThdSleepMilliseconds(5);
	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_tof_detection(void){
	chThdCreateStatic(waTofDetection, sizeof(waTofDetection), NORMALPRIO, TofDetection, NULL);
}
