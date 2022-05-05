#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <motors.h>
#include "main.h"

#include "proximity_detection.h"

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int proximity3_value = 0;

	while(1){

    	proximity3_value = PROX_FACTOR * get_calibrated_prox(2);

    	if(proximity3_value > GOAL_PROX_VALUE){
    		set_body_led(1);
    	}else{
    		set_body_led(0);
    	}

		chThdSleepMilliseconds(100);
	}
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

// sensor value in mm conversion
float conv_prox_mm(int error){
	return /*60 -*/ error * PROX_MM_FACTOR;
}

void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}
