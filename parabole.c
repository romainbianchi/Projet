#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <main.h>
#include "parabole.h"

#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92 //[step/cm]
#define GAP_WHEEL 5.3 //[cm]

float speed_conversion_cm_to_step(uint16_t cm_speed){
	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
}

float outer_speed(float inner_speed, float d){
	return (2*PI/d + 1)*inner_speed;
}

static THD_WORKING_AREA(waCircling, 128);
static THD_FUNCTION(Circling, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	float d = 5;			//defined in cm
	float speed = 7;		//defined in cm/s

	left_motor_set_speed(speed_conversion_cm_to_step(speed));
	right_motor_set_speed(speed_conversion_cm_to_step(outer_speed(speed, d)));


	while(1){
		chThdSleepMilliseconds(100);
	}
}


void start_circling(void){
	chThdCreateStatic(waCircling, sizeof(waCircling), NORMALPRIO, Circling, NULL);
}
