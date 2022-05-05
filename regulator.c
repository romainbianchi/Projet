#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <motors.h>

#include "regulator.h"
#include "main.h"
#include "motors.h"
#include "proximity_detection.h"

// pi regulator function
int16_t pi_regulator(int prox_value, int goal){

	float error = 0;
	float error_prev = 0;
	float prox = 0;
	static float sum_error = 0;
	float sub_error = 0;

	//chprintf((BaseSequentialStream *)&SD3, "prox_value␣=␣%f\r\n",conv_prox_mm(prox_value));

	error = conv_prox_mm(prox_value) - goal;
	chprintf((BaseSequentialStream *)&SD3, "error␣=␣%f\r\n", error);

	chprintf((BaseSequentialStream *)&SD3, "prox_value␣=␣%d\r\n", get_calibrated_prox(2));

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
	chprintf((BaseSequentialStream *)&SD3, "prox␣=␣%f\r\n",prox);
	chprintf((BaseSequentialStream *)&SD3, "KP*err␣=␣%f\r\n", KP*error);
	chprintf((BaseSequentialStream *)&SD3, "KI*sum_err␣=␣%f\r\n", KI*sum_error);
	chprintf((BaseSequentialStream *)&SD3, "KD*sub_error␣=␣%f\r\n", KD*sub_error);

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

    	prox = pi_regulator(get_calibrated_prox(2), GOAL_PROX_VALUE);

    	right_motor_set_speed(INITIAL_SPEED - prox);
    	left_motor_set_speed(INITIAL_SPEED);

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

//------------------------------------------------------------- EXTERNAL FUNCTIONS ----------------------------------------------------------------------

void start_regulator(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO + 1, PiRegulator, NULL);
}
