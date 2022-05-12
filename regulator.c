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
#include "motors.h"
#include "proximity_detection.h"
#include "TOF_detection.h"
#include "gravity_detection.h"
#include "parabole.h"

static uint8_t function_mode = NORMAL_FUNCTION_MODE;

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

//    	chprintf((BaseSequentialStream *)&SD3, "time␣=␣%dus\n",time);

    	if(get_selector() == SELECT_START){

//    		if(get_object_detected() && function_mode == NORMAL_FUNCTION_MODE){
//    			function_mode = ROTATION_FUNCTION_MODE;
//    		}
//
//    		if(get_angle() > ANGLE_PARABOLA - ANGLE_THRESHOLD && get_angle() < ANGLE_PARABOLA + ANGLE_THRESHOLD && function_mode == ROTATION_FUNCTION_MODE){
//    			function_mode = PARABOLA_FUNCTION_MODE;
//    		}
//
//    		if(get_floor_detected() && function_mode == PARABOLA_FUNCTION_MODE){
//    			function_mode = LANDING_FUNCTION_MODE;
//    		}
//    		if(get_angle() > ANGLE_HORIZONTAL - 3 && get_angle() < ANGLE_HORIZONTAL + 3 && function_mode == LANDING_FUNCTION_MODE){
//    			function_mode = NORMAL_FUNCTION_MODE;
//    			set_floor_detected(false);
//    		}




    		if(function_mode == ROTATION_FUNCTION_MODE){
    			rotation();
    		}
//    		if(function_mode == PARABOLA_FUNCTION_MODE){
////    			parabola();
//    		left_motor_set_speed(0);
//    		right_motor_set_speed(0);
//			}
    		if(function_mode == LANDING_FUNCTION_MODE){
    			rotation();
    		}
    		if(function_mode == NORMAL_FUNCTION_MODE){
    			prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);
    			right_motor_set_speed(INITIAL_SPEED + prox);
    			left_motor_set_speed(INITIAL_SPEED - prox);
    		}


    	}else{
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}

    	chThdSleepMilliseconds(10);
    	//chThdSleepUntilWindowed(time, time + MS2ST(2));
    }
}

//--------------------------------------- PUBLIC FUNCTIONS -----------------------------------------------------

void start_regulator(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+1, PiRegulator, NULL);
}

void set_function_mode(uint8_t mode){
	function_mode = mode;
}

uint8_t get_function_mode(void){
	return function_mode;
}
