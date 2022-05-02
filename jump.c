#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <main.h>
#include "jump.h"

#define	CONSTANT_CONVERSION_SPEED	0.013 //[cm/step]

//-------------------------------------------------------- INTERNAL FUNCTIONS --------------------------------------------------------------------------
float speed_conversion(uint16_t step_speed){
	return step_speed * CONSTANT_CONVERSION_SPEED;
}

/*static THD_WORKING_AREA(waSetSpeed, 128);
static THD_FUNCTION(SetSpeed, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	static uint32_t t = 0;
	static uint32_t speed = 0;

	while(1){
		speed = -GRAVITY*t+INITIAL_SPEED;

		left_motor_set_speed(speed);
		right_motor_set_speed(speed);

		if(abs(speed) < 250){
			speed = 0;
		}
    	t++;
		chThdSleepMilliseconds(100);
	}
}

//-------------------------------------------------------- EXTERNAL FUNCTIONS --------------------------------------------------------------------------
void start_speed(void){
	chThdCreateStatic(waSetSpeed, sizeof(waSetSpeed), NORMALPRIO, SetSpeed, NULL);
}*/

/*void set_speed(uint32_t time){
	static uint32_t speed = 0;
	speed = -GRAVITY*time+INITIAL_SPEED;

	left_motor_set_speed(speed);
	right_motor_set_speed(speed);

	if(abs(speed) < 250){
		speed = 0;
	}
}*/

