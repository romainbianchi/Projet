#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <main.h>
#include "parabole.h"
#include "regulator.h"
#include "selector.h"

#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92f //[step/cm]
#define GAP_WHEEL 5.3 //[cm]
#define VYO 10.0f //[cm/s]
#define VXO 3.0f
#define G -2.0f //[cm/s^2]
#define DT 0.02f  // [s]

//------------------------------- INTERNAL FUNCTIONS --------------------------------

//float speed_conversion_cm_to_step(float cm_speed){
//	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
//}
//
//float calculate_outer_speed(float roc, float v){
//	return v*(1+(GAP_WHEEL/(2*roc)));
//}
//float calculate_inner_speed(float roc, float v){
//	return v*(1-(GAP_WHEEL/(2*roc)));
//}
//
//float calculate_roc(float t){
//	return (float)pow(pow(VXO*VXO + (G*t+VYO)*(G*t+VYO), 1.0/2.0), 3.0)/abs(VXO*G);
//}
//
//float calculate_norm_speed(float t){
//	return sqrt((G*t)*(G*t)+(2*G*VYO*t)+VXO*VXO+VYO*VYO);
//}

//static THD_WORKING_AREA(waParabola, 256);
//static THD_FUNCTION(Parabola, arg){
//
//	chRegSetThreadName(__FUNCTION__);
//	(void)arg;
//
//	static float t = 0;				//Time, defined in s
//	static float roc = 0;			//Radius of Curvature, defined in cm
//	static float speed = 0; 		//Initial Speed, defined in cm/s
//
//	while(1){
//
//		if(get_selector() == SELECT_START){
//			if(get_function_mode() == PARABOLA_FUNCTION_MODE){
//				roc = calculate_roc(t);
//				speed = calculate_norm_speed(t);
//
//				left_motor_set_speed((int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed)));
//				right_motor_set_speed((int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed)));
//
//				t=t+DT;
//			}
//		}
//
//		chThdSleepMilliseconds(DT*1000);
//	}
//}

//------------------------------- EXTERNAL FUNCTIONS -------------------------------

//void start_parabola(void){
//	chThdCreateStatic(waParabola, sizeof(waParabola), NORMALPRIO+1, Parabola, NULL);
//}

void rotation(void){
	left_motor_set_speed(-ROTATION_SPEED);
	right_motor_set_speed(ROTATION_SPEED);
}

void stop_rotation(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

//void parabola(float time_parabola){
//	static float t = 0;
//	static float roc = 0;
//	static float speed = 0;
//
//	roc = calculate_roc(time_parabola);
//	speed = calculate_norm_speed(time_parabola);
//	left_motor_set_speed((int)speed_conversion_cm_to_step(calculate_outer_speed(roc, speed)));
//	right_motor_set_speed((int)speed_conversion_cm_to_step(calculate_inner_speed(roc, speed)));
//
//
//	t=t+DT;
//}
