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

//PARABOLA CONSTANTS
#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP		76.92f 		//[step/cm]
#define GAP_WHEEL 									5.3 		//[cm]
#define	JUMP_VYO 									18.0f 		//[cm/s]
#define FALL_VYO									0.0f		//[cm/s]
#define VXO 										6.0f		//[cm/s]
#define G 											-8.0f		//[cm/s^2]
#define DT 											0.02f 		//[s]

//ROTATIION CONSTANTS
#define ROTATION_SPEED								180			//[steps/s]
#define CLOCKWISE									0
#define ANTI_CLOCKWISE								1

//REGULATOR CONSTANTS
#define INITIAL_SPEED								600 		//[steps/s]
#define PROX_FACTOR									0.01f
#define GOAL_PROX_VALUE								1000.00f * PROX_FACTOR
#define KP 											5.0f
#define KI											0.02f
#define KD											250.0f
#define MAX_SUM_ERROR 								300
#define ERROR_THRESHOLD								0.0


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

float calculate_roc(float t, float vyo){
	return (float)pow(pow(VXO*VXO + (G*t+vyo)*(G*t+vyo), 1.0/2.0), 3.0)/abs(VXO*G);
}

float calculate_norm_speed(float t, float vyo){
	return sqrt((G*t)*(G*t)+(2*G*vyo*t)+VXO*VXO+vyo*vyo);
}

void parabola(float vyo){
	static float roc = 0;
	static float speed = 0;

	roc = calculate_roc(time_parabola, vyo);
	speed = calculate_norm_speed(time_parabola, vyo);
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

    	if(get_selector() == SELECT_START){

    		switch(get_function_mode()){

    			case NORMAL_FUNCTION_MODE:
    				prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);
    				right_motor_set_speed(INITIAL_SPEED + prox);
    				left_motor_set_speed(INITIAL_SPEED - prox);
    				break;

    			case PARABOLA_FUNCTION_MODE:
        			parabola(JUMP_VYO);
        			break;

    			case ROTATION_FUNCTION_MODE:
        			rotation(ANTI_CLOCKWISE);
        			break;

    			case INV_ROTATION_FUNCTION_MODE:
        			rotation(CLOCKWISE);
        			break;

    			case LANDING_FUNCTION_MODE:
        			time_parabola = 0;
        			rotation(ANTI_CLOCKWISE);
        			break;

    			case FALL_FUNCTION_MODE:
        			parabola(FALL_VYO);
        			break;

    			case END_FUNCTION_MODE:
    				stop_motors();
    				set_body_led(1);
    				break;
    		}

    	}else{
    		stop_motors();
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    }
}

//--------------------------------------- PUBLIC FUNCTIONS -----------------------------------------------------

void start_regulator(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+1, PiRegulator, NULL);
}

void rotation(uint8_t direction){
	if(direction == ANTI_CLOCKWISE){
		left_motor_set_speed(-ROTATION_SPEED);
		right_motor_set_speed(ROTATION_SPEED);
	}else if(direction == CLOCKWISE){
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
	}
}

void stop_motors(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
