#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <selector.h>

#include "main.h"
#include "regulator.h"
#include "main.h"
#include "motors.h"
#include "proximity_detection.h"

//--------------------------------------- INTERNAL FUNCTIONS -----------------------------------------------------

int16_t pi_regulator(int prox_value, int goal){

	float error = 0;
	static float error_prev = 0;
	float prox = 0;
	static float sum_error = 0;
	float sub_error = 0;

	error = (PROX_FACTOR * prox_value) - goal;

	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	sub_error = error - error_prev;

	prox = KP * error + KI * sum_error + KD * sub_error;

	error_prev = error;

	return (int16_t)prox;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = 0;

    int16_t prox = 0;

    while(1){

    	time = chVTGetSystemTime();

    	if(get_selector() == SELECT_START){

    		if(get_object_detected()){
    			right_motor_set_speed(0);
    			left_motor_set_speed(0);
    		}else{
    			prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);
    			right_motor_set_speed(INITIAL_SPEED + prox);
    			left_motor_set_speed(INITIAL_SPEED - prox);
    		}

    	}else{
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(2));
    }
}

//--------------------------------------- PUBLIC FUNCTIONS -----------------------------------------------------

void start_regulator(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
