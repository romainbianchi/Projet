#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <selector.h>
#include <leds.h>

#include "main.h"
#include "motors_control.h"

//PARABOLA CONSTANTS
#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP		76.92f 		//[step/cm]
#define GAP_WHEEL 									5.3 		//[cm]
#define	JUMP_VYO 									18.0f 		//[cm/s]
#define FALL_VYO									0.0f		//[cm/s]
#define JUMP_VXO 									6.0f		//[cm/s]
#define G 											-8.0f		//[cm/s^2]
#define DT 											0.02f 		//[s]

//ROTATIION CONSTANTS
#define ROTATION_SPEED								2.34f		//[cm/s]
#define CLOCKWISE									0
#define COUNTER_CLOCKWISE							1

//REGULATOR CONSTANTS
#define INITIAL_SPEED								8.0f		//[cm/s]
#define PROX_FACTOR									0.01f
#define GOAL_PROX_VALUE								1000.00f * PROX_FACTOR
#define KP 											5.0f
#define KI											0.02f
#define KD											230.0f
#define MAX_SUM_ERROR 								300
#define ERROR_THRESHOLD								0.0


static float time_parabola = 0;

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

//PARABOLE

/* Conversion from cm to step for the motors functions */
float speed_conversion_cm_to_step(float cm_speed){
	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
}

/* Return the radius of curvature of the parabola at time t*/
float calculate_roc(float t, float vyo, float vxo){
	return (float)pow(pow(vxo*vxo + (G*t+vyo)*(G*t+vyo), 1.0/2.0), 3.0)/abs(vxo*G);
}

/* Return the norm of the tangential speed of the parabola at time t */
float calculate_norm_speed(float t, float vyo, float vxo){
	return sqrt((G*t)*(G*t)+(2*G*vyo*t)+vxo*vxo+vyo*vyo);
}

/* Return the speed of the outer wheel in order to follow the parabola */
float calculate_outer_speed(float roc, float v){
	return v*(1+(GAP_WHEEL/(2*roc)));
}

/* Return the speed of the inner wheel in order to follow the parabola */
float calculate_inner_speed(float roc, float v){
	return v*(1-(GAP_WHEEL/(2*roc)));
}

/* make the robot follow the desired parabola
 * DT must be accorded with the timing of the motors_control thread in order to obtain the correct shape of the parabola
 * The time is incremented every time the function is called in the thread
 */
void parabola(float vyo, float vxo){
	static float roc = 0;
	static float speed = 0;

	roc = calculate_roc(time_parabola, vyo, vxo);
	speed = calculate_norm_speed(time_parabola, vyo, vxo);
	left_motor_set_speed((int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed)));
	right_motor_set_speed((int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed)));


	time_parabola = time_parabola+DT;
}


//REGULATOR

/* PID regulator function to maintain the robot at the desired value from the wall */
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

/* Motors control thread
 * Change the motors behavior according to the actual mode
 */
static THD_WORKING_AREA(waMotorControl, 256);
static THD_FUNCTION(MotorControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = 0;

    int16_t prox = 0;


    while(1){

    	time = chVTGetSystemTime();

    	if(get_selector() == SELECT_START){

    		switch(get_function_mode()){

    			case NORMAL_MODE:
    				prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);
    				right_motor_set_speed((int)speed_conversion_cm_to_step(INITIAL_SPEED) + prox);
    				left_motor_set_speed((int)speed_conversion_cm_to_step(INITIAL_SPEED) - prox);
    				break;

    			case PARABOLA_MODE:
        			parabola(JUMP_VYO, JUMP_VXO);
        			break;

    			case ROTATION_MODE:
        			rotation(COUNTER_CLOCKWISE);
        			break;

    			case INV_ROTATION_MODE:
        			rotation(CLOCKWISE);
        			break;

    			case LANDING_MODE:
        			time_parabola = 0;
        			rotation(COUNTER_CLOCKWISE);
        			break;

    			case INV_LANDING_MODE:
    				time_parabola = 0;
    				rotation(CLOCKWISE);
    				break;

    			case FALL_MODE:
        			parabola(FALL_VYO, INITIAL_SPEED);
        			break;

    			case CONTROL_PARA_ANGLE_MODE:
    				stop_motors();
    				break;

    			case CONTROL_HORZ_ANGLE_MODE:
    				stop_motors();
    				break;

    			case END_MODE:
    				stop_motors();
    				set_body_led(1);
    				break;
    		}

    	}else{
    		stop_motors();
    	}

    	//chThdSleepUntilWindowed needed
    	//assure that the parabola function is called at the same interval an so that the time is correctly incremented
    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    }
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

/* Start motor control thread*/
void start_motors_control(void){
	chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO+1, MotorControl, NULL);
}

/* clockwise or counter clockwise rotation */
void rotation(uint8_t direction){
	if(direction == COUNTER_CLOCKWISE){
		left_motor_set_speed(-(int)speed_conversion_cm_to_step(ROTATION_SPEED));
		right_motor_set_speed((int)speed_conversion_cm_to_step(ROTATION_SPEED));
	}else if(direction == CLOCKWISE){
		left_motor_set_speed((int)speed_conversion_cm_to_step(ROTATION_SPEED));
		right_motor_set_speed(-(int)speed_conversion_cm_to_step(ROTATION_SPEED));
	}
}

/* stop the motors */
void stop_motors(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
