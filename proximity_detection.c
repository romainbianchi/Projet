#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include "main.h"

#include "proximity_detection.h"

static bool object_detected = false;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int proximity3_value = 0;
	int proximity6_value = 0;
	int TOF_value = 0;

	systime_t time = 0;

	while(1){

		time = chVTGetSystemTime();

    	proximity3_value = PROX_FACTOR * get_calibrated_prox(2);
    	TOF_value = VL53L0X_get_dist_mm();

    	if(TOF_value < 120){
    		set_front_led(1);
    		object_detected = true;
    		/*left_motor_set_speed(300);
    		right_motor_set_speed(-300);
    		do{
    			proximity6_value = PROX_FACTOR * get_calibrated_prox(5);
    		}while(proximity6_value < GOAL_PROX_VALUE);*/
    	}else{
    		set_front_led(0);
    		object_detected = false;
    	}

    	if(proximity3_value > GOAL_PROX_VALUE){
    		set_body_led(1);
    	}else{
    		set_body_led(0);
    	}

	}
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}

bool get_object_detected(void){
	return object_detected;
}
