#include <ch.h>
#include <hal.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <selector.h>

#include "main.h"
#include "TOF_detection.h"

#define GOAL_OBJECT_VALUE 		100 //[mm]

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------


/* TOF detection thread
 * Switch between normal mode and rotation mode when an object is detected
 */
static THD_WORKING_AREA(waTofDetection, 128);
static THD_FUNCTION(TofDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){

		if(get_selector() ==  SELECT_START){
			if(VL53L0X_get_dist_mm() < GOAL_OBJECT_VALUE && get_function_mode() == NORMAL_MODE){
				set_function_mode(ROTATION_MODE);
			}
		}

		chThdSleepMilliseconds(5);
	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

void start_tof_detection(void){
	chThdCreateStatic(waTofDetection, sizeof(waTofDetection), NORMALPRIO, TofDetection, NULL);
}
