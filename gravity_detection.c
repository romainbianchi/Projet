#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <main.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <motors.h>
#include <selector.h>

#include "regulator.h"
#include "TOF_detection.h"
#include "gravity_detection.h"

#define	ANGLE_AT_NINETY			100000
#define ANGLE_PARABOLA_SUPP		4.2f
#define ANGLE_PARABOLA_INF		2.3f
#define ANGLE_LANDING_SUPP		0.09f
#define ANGLE_LANDING_INF		-0.09f
#define COUNT_BEFORE_CONTROL	10


static float angle_from_horizontal = 0; // représente tan(alpha), rapport des valeurs de l'imu
static uint8_t quadrant = 0; // dans quel quadrant pointe le robot


//------------------------------- INTERNAL FUNCTIONS --------------------------------

/* Use the IMU acceleration values to calculate the angle between the horizontal and the front of the robot
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


/* Gravity detection thread
 * Detect the desired angle according to the actual mode
 * Due to the instability during rotation, the angle is controlled before starting the parabola
 */
static THD_WORKING_AREA(waGravity, 512);
static THD_FUNCTION(Gravity, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    static uint8_t count_parabola = 0;
    static uint8_t count_landing = 0;

	while(1){

    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	determine_angle(imu_values);

    	if(get_selector() ==  SELECT_START){


    		// Detect the parabola angle when the robot is in rotation or inverse rotation mode
			if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP && quadrant == 1
					&& (get_function_mode() == ROTATION_FUNCTION_MODE || get_function_mode() == INV_ROTATION_FUNCTION_MODE)){

				stop_motors();
				set_function_mode(CONTROL_PARA_ANGLE_FUNCTION_MODE);

			}


			// When in landing mode or inverse landing mode, detect if the robot is in the horizontal range defined
    		if(angle_from_horizontal > ANGLE_LANDING_INF && angle_from_horizontal < ANGLE_LANDING_SUPP
    				&& (get_function_mode() == LANDING_FUNCTION_MODE || get_function_mode() == INV_LANDING_FUNCTION_MODE)
					&& (quadrant == 1 || quadrant == 4)){

    			stop_motors();
    			set_function_mode(CONTROL_HORZ_ANGLE_FUNCTION_MODE);

    		}


			// when in control angle mode, control if the robot is in the desired range before starting the parabola
			// if not, adapt the direction of rotation to approach the desired angle.
			if(get_function_mode() == CONTROL_PARA_ANGLE_FUNCTION_MODE){

				if(count_parabola == COUNT_BEFORE_CONTROL){

					count_parabola = 0;
					determine_angle(imu_values);

					if(angle_from_horizontal > ANGLE_PARABOLA_INF && angle_from_horizontal < ANGLE_PARABOLA_SUPP && quadrant == 1){

						set_function_mode(PARABOLA_FUNCTION_MODE);

					// if the robot is in the first quadrant, determine the angle is larger or smaller than the desired range and adapt rotation
					}else if(quadrant == 1){

						if(angle_from_horizontal < ANGLE_PARABOLA_INF){
							set_function_mode(ROTATION_FUNCTION_MODE);
						}else if(angle_from_horizontal > ANGLE_PARABOLA_SUPP){
							set_function_mode(INV_ROTATION_FUNCTION_MODE);
						}

					// if the front of the robot is in the second quadrant, the angle is to big, adapt the rotation
					}else if(quadrant == 2){

						set_function_mode(INV_ROTATION_FUNCTION_MODE);

					}

				}else{

					count_parabola++;

				}
			}


			// When in control horz angle mode, control if the robot is in the horz range defined
			// If not, adapt the direction of rotation to approach the desired angle
			if(get_function_mode() == CONTROL_HORZ_ANGLE_FUNCTION_MODE){

				if(count_landing == COUNT_BEFORE_CONTROL){

					count_landing = 0;
					determine_angle(imu_values);

					if(angle_from_horizontal > ANGLE_LANDING_INF && angle_from_horizontal < ANGLE_LANDING_SUPP
							&& (quadrant == 1 || quadrant == 4)){

						set_function_mode(NORMAL_FUNCTION_MODE);

					}else if(angle_from_horizontal < ANGLE_LANDING_INF){

						set_function_mode(LANDING_FUNCTION_MODE);

					}else if(angle_from_horizontal > ANGLE_LANDING_SUPP || quadrant == 2 || quadrant == 3){

						set_function_mode(INV_LANDING_FUNCTION_MODE);

					}
				}else{

					count_landing++;

				}

			}


    	}

    	chThdSleepMilliseconds(5);
	}
}

//------------------------------- EXTERNAL FUNCTIONS -------------------------------

/* Start gravity detection thread */
void start_gravity(void){
	chThdCreateStatic(waGravity, sizeof(waGravity), NORMALPRIO+1, Gravity, NULL);
}
