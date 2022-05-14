#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <selector.h>

#include "main.h"
#include "proximity_detection.h"
#include "regulator.h"

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

bool detect_fall(void){
	if(get_calibrated_prox(1) < 10){
		return true;
	}else{
		return false;
	}
}

static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){

		if(get_selector() == SELECT_START){
			if(get_calibrated_prox(1) < 10 && get_function_mode() == NORMAL_FUNCTION_MODE){
				set_function_mode(FALL_FUNCTION_MODE);
			}
		}

		chThdSleepMilliseconds(20);
		//chThdSleepUntilWindowed(time, time + MS2ST(5));
	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}
