#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <main.h>
#include "parabole.h"

#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92f //[step/cm]
#define GAP_WHEEL 5.3 //[cm]
#define VYO 4.0f //[cm/s]
#define VXO 4.0f
#define G -0.5f //[cm/s^2]
#define DT 0.2f  // [s]



static THD_WORKING_AREA(waParabola, 128);
static THD_FUNCTION(Parabola, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	static float t = 0;				//Time, defined in s
	static float roc = 0;			//Radius of Curvature, defined in cm
	static float speed = 0; 		//Initial Speed, defined in cm/s

	while(1){

		roc = calculate_roc(t);
		speed = calculate_norm_speed(t);
//		roc = d_roc(t);
//		speed = d_norm_speed(t);
//		volatile float debug = atanf((y(t+DT)-y(t))/(DT))*180/PI;
//		volatile int v = (int)speed_conversion_cm_to_step(speed);

//		volatile int vout = (int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed));
//		volatile int vint = (int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed));

		if(t>-1.0){
			left_motor_set_speed((int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed)));
			right_motor_set_speed((int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed)));
		} else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}


//		if(calculate_inner_speed(roc, speed) < 0){
//			right_motor_set_speed(0);
//		}

		t=t+DT;
		chThdSleepMilliseconds(DT*1000);
	}
}


void start_parabola(void){
	chThdCreateStatic(waParabola, sizeof(waParabola), NORMALPRIO, Parabola, NULL);
}

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


//discretisation--------------------------------------------

float x(float t){								//x(t)
	return VXO*t;
}

float d1d_x(float t){					//Discrete First Derivative for x(t) -> v_x(t)
	return (x(t+DT)-x(t))/(DT);
}

float d2d_x(float t){					// Discrete Second Derivative for x(t) -> a_x(t)
	return (x(t+DT)+x(t-DT)-2*x(t))/(DT*DT);
}

float y(float t){								// y(t)
	return (G*t*t)+(VYO*t);
}

float d1d_y(float t){					//Discrete First Derivative for y(t) -> v_y(t)
	return (y(t+DT)-y(t))/(DT);
}

float d2d_y(float t){					// Discrete Second Derivative for y(t) -> a_y(t)
	return (y(t+DT)+y(t-DT)-2*y(t))/(DT*DT);
}

float d_roc(float t){
	return (float)pow((d1d_x(t))*(d1d_x(t)) + (d1d_y(t))*(d1d_y(t)), 3/2)/abs((d1d_x(t))*(d2d_y(t))-(d1d_y(t))*(d2d_x(t)));
}

float d_norm_speed(float t){
	return (float)sqrt(d1d_x(t)*d1d_x(t) + d1d_y(t)*d1d_y(t));
}

float d_angle(float t){
	return (float)acos(DT*d1d_x(t)/d1d_y(t));
}


