#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <main.h>
#include "parabole.h"

#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92 //[step/cm]
#define GAP_WHEEL 5.3 //[cm]
#define V0 3//[cm/s]
#define VX 8
#define G 500//[cm/s^2]



static THD_WORKING_AREA(waParabola, 128);
static THD_FUNCTION(Parabola, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	static float t = 0;			//Time, defined in s
	static float roc = 0;			//Radius of Curvature, defined in cm
//	static float speed = 0; 		//Initial Speed, defined in cm/s

	//left_motor_set_speed(speed_conversion_cm_to_step(speed));
	//right_motor_set_speed(speed_conversion_cm_to_step(outer_speed(speed, d)));


	while(1){
		roc = calculate_roc(t);

		left_motor_set_speed(speed_conversion_cm_to_step(calculate_outer_speed(roc)));
		right_motor_set_speed(speed_conversion_cm_to_step(calculate_inner_speed(roc)));
		t++;
		chThdSleepMilliseconds(100);
	}
}


void start_parabola(void){
	chThdCreateStatic(waParabola, sizeof(waParabola), NORMALPRIO, Parabola, NULL);
}

float speed_conversion_cm_to_step(float cm_speed){
	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
}

/*float outer_speed(float inner_speed, float d){
	return (2*PI/d + 1)*inner_speed;
}*/
float calculate_outer_speed(float roc){
	return VX*(1+(GAP_WHEEL/(2*roc)));
}
float calculate_inner_speed(float roc){
	//return sqrt((G*G*t*t/1000000)-(1.41*G*V0*t/1000)+(V0*V0));
	return VX*(1-(GAP_WHEEL/(2*roc)));
}

float calculate_roc(float t){
	return pow((1+(V0*V0)-(4*G*V0*t)+(2*G*t*G*t)), 3/2)/(2*G);
}
