#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>

#include "main.h"
#include "proximity_detection.h"

static bool floor_detected = false;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int proximity2_value = 0;
	int proximity3_value = 0;

	systime_t time = 0;

	while(1){

		time = chVTGetSystemTime();

		proximity2_value = PROX_FACTOR * get_calibrated_prox(1);
    	proximity3_value = PROX_FACTOR * get_calibrated_prox(2);

    	if(proximity3_value > GOAL_PROX_VALUE || proximity2_value > GOAL_PROX_VALUE){
    		set_body_led(1);
    		floor_detected = true;
    	}else{
    		set_body_led(0);
    	}

	}
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}

bool get_floor_detected(void){
	return floor_detected;
}
