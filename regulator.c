#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <selector.h>
#include <sensors/imu.h>
#include <leds.h>

#include "main.h"
#include "regulator.h"
#include "motors.h"
#include "proximity_detection.h"
#include "TOF_detection.h"
#include "gravity_detection.h"
#include "parabole.h"

#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92f //[step/cm]
#define GAP_WHEEL 5.3 //[cm]
#define VYO 10.0f //[cm/s]
#define VXO 3.0f
#define G -2.0f //[cm/s^2]
#define DT 0.02f  // [s]

static uint8_t function_mode = NORMAL_FUNCTION_MODE;
static float time_parabola = 0;

//--------------------------------------- INTERNAL FUNCTIONS -----------------------------------------------------

//PARABOLE
float speed_conversion_cm_to_step(float cm_speed){
	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
}

float calculate_outer_speed(float roc, float v){
	return v*(1+(GAP_WHEEL/(2*roc)));
}
float calculate_inner_speed(float roc, float v){
	return v*(1-(GAP_WHEEL/(2*roc)));
}

float calculate_roc(float t){
	return (float)pow(pow(VXO*VXO + (G*t+VYO)*(G*t+VYO), 1.0/2.0), 3.0)/abs(VXO*G);
}

float calculate_norm_speed(float t){
	return sqrt((G*t)*(G*t)+(2*G*VYO*t)+VXO*VXO+VYO*VYO);
}

void parabola(){
	static float roc = 0;
	static float speed = 0;

	roc = calculate_roc(time_parabola);
	speed = calculate_norm_speed(time_parabola);
	left_motor_set_speed((int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed)));
	right_motor_set_speed((int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed)));


	time_parabola = time_parabola+DT;
}

//REGULATOR
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

//THREAD
static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = 0;

    int16_t prox = 0;

    while(1){

    	time = chVTGetSystemTime();

		//chprintf((BaseSequentialStream *)&SD3, "angle␣=␣%f\r\n",get_angle());

    	if(get_selector() == SELECT_START){

    		if(function_mode == ROTATION_FUNCTION_MODE){
    			rotation();
    		}
    		if(function_mode == INV_ROTATION_FUNCTION_MODE){
    			left_motor_set_speed(ROTATION_SPEED);
    			right_motor_set_speed(-ROTATION_SPEED);
    		}
    		if(function_mode == PARABOLA_FUNCTION_MODE){
    			parabola(time_parabola);
			}
    		if(function_mode == LANDING_FUNCTION_MODE){
    			time_parabola = 0;
    			rotation();
    		}
    		if(function_mode == NORMAL_FUNCTION_MODE){
    			prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);
    			right_motor_set_speed(INITIAL_SPEED + prox);
    			left_motor_set_speed(INITIAL_SPEED - prox);
    		}
    		if(function_mode == FALL_FUNCTION_MODE){
    			set_body_led(1);
    		}

    	}else{
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}

//    	chThdSleepMilliseconds(10);
    	chThdSleepUntilWindowed(time, time + MS2ST(20));
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
