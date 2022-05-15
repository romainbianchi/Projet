#include <ch.h>
#include <hal.h>
#include <sensors/imu.h>
#include <selector.h>

#include "gravity_detection.h"
#include "main.h"

#define	ANGLE_AT_NINETY			100000
#define ANGLE_PARABOLA_SUPP		4.2f	// 76.6°
#define ANGLE_PARABOLA_INF		2.3f	// 68.5°
#define ANGLE_LANDING_SUPP		0.09f	// 5.1°
#define ANGLE_LANDING_INF		-0.09f	// -5.1°
#define COUNT_BEFORE_CONTROL	10


static float angle_from_horizontal = 0;
static uint8_t quadrant = 0;
static uint8_t count_parabola = 0;
static uint8_t count_landing = 0;


//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------

/* Use the IMU acceleration values to calculate the angle between the horizontal and the front of the robot
 * The angle is represented by the division of the y acceleration value and the x acceleration value
 * Since the angle wanted is known we can use this result to find an angle by doing the arctan manually and make the code more efficient
 * Also determine in which trigonometrical quadrant is the front of the robot pointing
 */
void determine_angle(imu_msg_t imu_values){
	float *accel = imu_values.acceleration;

	if(accel[X_AXIS] < 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 1;
		}else{
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 4;
		}
	}else if(accel[X_AXIS] > 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 2;
		}else{
			angle_from_horizontal = accel[Y_AXIS]/(-accel[X_AXIS]);
			quadrant = 3;
		}
	//Security to be sure the division doesn't go to infinity
	}else if(accel[X_AXIS] == 0){
		if(accel[Y_AXIS] > 0){
			angle_from_horizontal = ANGLE_AT_NINETY;
			quadrant = 1;
		}else{
			angle_from_horizontal = -ANGLE_AT_NINETY;
			quadrant = 4;
		}
	}
}

/* Detect if the robot is in the parabola range defined */
void search_parabola_angle(void){
	if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP
			&& quadrant == 1){
		set_function_mode(CONTROL_PARA_ANGLE_MODE);
	}
}

/* Detect if the robot is in the horizontal range defined */
void search_landing_angle(void){
	if(angle_from_horizontal > ANGLE_LANDING_INF && angle_from_horizontal < ANGLE_LANDING_SUPP
			&& (quadrant == 1 || quadrant == 4)){
		set_function_mode(CONTROL_HORZ_ANGLE_MODE);
	}
}


/* Control if the robot is in the desired range before starting the parabola
 * If not, adapt the direction of rotation to approach the desired angle.
 */
void control_parabola_angle(imu_msg_t imu_values){

	if(count_parabola == COUNT_BEFORE_CONTROL){

		count_parabola = 0;
		determine_angle(imu_values);

		if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP && quadrant == 1){

			set_function_mode(PARABOLA_MODE);

		// if the robot is in the first quadrant, determine the angle is larger or smaller than the desired range and adapt rotation
		}else if(quadrant == 1){

			if(angle_from_horizontal < ANGLE_PARABOLA_INF){
				set_function_mode(ROTATION_MODE);
			}else if(angle_from_horizontal > ANGLE_PARABOLA_SUPP){
				set_function_mode(INV_ROTATION_MODE);
			}

		// if the front of the robot is in the second quadrant, the angle is to big, adapt the rotation
		}else if(quadrant == 2){
			set_function_mode(INV_ROTATION_MODE);
		}

	}else{
		count_parabola++;
	}
}


/* Control if the robot is in the horz range defined
 * If not, adapt the direction of rotation to approach the desired angle
 */
void control_landing_angle(imu_msg_t imu_values){

	if(get_function_mode() == CONTROL_HORZ_ANGLE_MODE){

		if(count_landing == COUNT_BEFORE_CONTROL){

			count_landing = 0;
			determine_angle(imu_values);

			if(angle_from_horizontal > ANGLE_LANDING_INF && angle_from_horizontal < ANGLE_LANDING_SUPP
					&& (quadrant == 1 || quadrant == 4)){
				set_function_mode(NORMAL_MODE);
			}else if(angle_from_horizontal < ANGLE_LANDING_INF){
				set_function_mode(LANDING_MODE);
			}else if(angle_from_horizontal > ANGLE_LANDING_SUPP || quadrant == 2 || quadrant == 3){
				set_function_mode(INV_LANDING_MODE);
			}

		}else{
			count_landing++;
		}
	}
}


/* Gravity detection thread
 * Detect the desired angle according to the actual mode
 * The angle is controlled before starting the parabola or returning to normal mode
 */
static THD_WORKING_AREA(waGravity, 512);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

	while(1){

    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	determine_angle(imu_values);

    	if(get_selector() ==  SELECT_START){

    		switch(get_function_mode()){

    			case ROTATION_MODE:
    				search_parabola_angle();
    				break;


    			case INV_ROTATION_MODE:
    				search_parabola_angle();
    				break;

    			case LANDING_MODE:
    				search_landing_angle();
    				break;

    			case INV_LANDING_MODE:
    				search_landing_angle();
    				break;

    			case CONTROL_PARA_ANGLE_MODE:
    				control_parabola_angle(imu_values);
    				break;

    			case CONTROL_HORZ_ANGLE_MODE:
    				control_landing_angle(imu_values);
    				break;
    		}

    	}

    	chThdSleepMilliseconds(5);
	}
}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------
/* Start gravity detection thread */
void start_gravity(void){
	chThdCreateStatic(waGravity, sizeof(waGravity), NORMALPRIO+1, Gravity, NULL);
}
