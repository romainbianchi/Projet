#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>

#include "main.h"
#include "proximity_detection.h"
#include "regulator.h"

static bool on_floor = false;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int proximity1_value = 0;
	int proximity2_value = 0;
	int proximity3_value = 0;

	systime_t time = 0;

	while(1){

		time = chVTGetSystemTime();

		proximity1_value = PROX_FACTOR * get_calibrated_prox(0);
		proximity2_value = PROX_FACTOR * get_calibrated_prox(1);
    	proximity3_value = PROX_FACTOR * get_calibrated_prox(2);

    	if(proximity3_value > GOAL_PROX_VALUE){
    		set_body_led(1);
    		on_floor = true;
    	}else{
    		set_body_led(0);
    		on_floor = false;

    	}

    	if(get_function_mode() == NORMAL_FUNCTION_MODE && !on_floor){
    		set_function_mode(FALL_FUNCTION_MODE);
    	}

		chThdSleepUntilWindowed(time, time + MS2ST(5));
	}
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}
