//#include <astolfix.h>
//#include <ch.h>
//#include <hal.h>
//#include <stdlib.h>
//#include <math.h>
//#include <motors.h>
//#include <main.h>
//
//#define WHEEL_RADIUS 2.05f // [cm]
//#define WHEEL_GAP 5.3f // [cm]
//#define THETA_START 45.0f
//#define VO 6.0f
//#define K_RHO 0.1f
//#define K_A 0.2f
//#define K_B -0.f
//
//#define DX 5.0f
//#define DY 5.0f
//
//#define	CONSTANT_CONVERSION_SPEED_CM_TO_STEP	76.92f //[step/cm]
//


//static THD_WORKING_AREA(waAstolfi, 254);
//static THD_FUNCTION(Astolfi, arg){
//
//	chRegSetThreadName(__FUNCTION__);
//	(void)arg;
//
//	static float trans_speed = VO;//determine_trans_speed(VO, VO);  // Initial speed V0, defined
//	static float rot_speed = 0; //determine_rot_speed(VO, VO);		// Initial rotational speed, =0
//	static float left_speed = VO;
//	static float right_speed = VO;
//	static float rho = sqrt(DX*DX+DY*DY);
//	static float alpha = -THETA_START*(3.14/180.0)+atan2(DY,DX);
//	static float beta = atan2(DY,DX);
//
//
//	while(1){
//
//		trans_speed = determine_trans_speed(left_speed, right_speed);
//		rot_speed = determine_rot_speed(left_speed, right_speed);
//
//		do_astolfi(&trans_speed, &rot_speed, rho, alpha, beta);
//
//		left_speed = calculate_outer_speed(rot_speed, trans_speed);
//		right_speed = calculate_inner_speed(rot_speed, trans_speed);
//
//		right_motor_set_speed((int)speed_conversion_cm_to_step(right_speed));
//		left_motor_set_speed((int)speed_conversion_cm_to_step(left_speed));
//
//		chThdSleepMilliseconds(10);
//	}
//}

//void start_astolfi(void){
//	chThdCreateStatic(waAstolfi, sizeof(waAstolfi), NORMALPRIO, Astolfi, NULL);
//}
//
//float determine_trans_speed(float left_speed, float right_speed){
//	return (left_speed+right_speed)/2;
//}
//
//float determine_rot_speed(float left_speed, float right_speed){
//	return (left_speed-right_speed)/WHEEL_GAP;
//}
//
//float calculate_outer_speed(float omega, float v){
//	return v+omega*WHEEL_GAP/2;
//}
//
//float calculate_inner_speed(float omega, float v){
//	return v-omega*WHEEL_GAP/2;
//}

//
//
//void do_astolfi(float* v, float* omega, float rho, float alpha, float beta){
//	*v = K_RHO*rho;
//	*omega = K_A*alpha+K_B*beta;
//}
//
//float speed_conversion_cm_to_step(float cm_speed){
//	return cm_speed * CONSTANT_CONVERSION_SPEED_CM_TO_STEP;
//}


