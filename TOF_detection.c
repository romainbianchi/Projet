#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include "main.h"
#include "TOF_detection.h"

static bool object_detected = false;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waTofDetection, 128);
static THD_FUNCTION(TofDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int TOF_value = 0;

	systime_t time = 0;

	while(1){
		time = chVTGetSystemTime();

		TOF_value = VL53L0X_get_dist_mm();

    	if(TOF_value < GOAL_TOF_VALUE){
    		set_front_led(1);
    		object_detected = true;
    	}else{
    		set_front_led(0);
    		object_detected = false;
    	}

	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_tof_detection(void){
	chThdCreateStatic(waTofDetection, sizeof(waTofDetection), NORMALPRIO, TofDetection, NULL);
}

bool get_object_detected(void){
	return object_detected;
}
