#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <selector.h>

#include "main.h"
#include "proximity_detection.h"
#include "regulator.h"

#define PROX_FALL_VALUE			10
#define PROX_FLOOR_DETECT		600
#define PROX_END_DETECT			200

//----------------------------------------------------- INTERNAL FUNCTIONS ------------------------------------------------------------------------------


/* Proximity detection thread
 * Switch between modes according to actual mode and proximity sensors values
 */
static THD_WORKING_AREA(waProximityDetection, 128);
static THD_FUNCTION(ProximityDetection, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){

		if(get_selector() == SELECT_START){

			// switch from normal mode to fall mode if no floor detected by IR2
			if(get_calibrated_prox(1) < PROX_FALL_VALUE && get_function_mode() == NORMAL_FUNCTION_MODE){
				set_function_mode(FALL_FUNCTION_MODE);
			}

			// switch from parabola or fall mode to landing mode if floor detected by IR1 or IR2
			if((get_calibrated_prox(0) > PROX_FLOOR_DETECT || get_calibrated_prox(1) > PROX_FLOOR_DETECT)
				&& (get_function_mode() == FALL_FUNCTION_MODE || get_function_mode() == PARABOLA_FUNCTION_MODE)){
				set_function_mode(LANDING_FUNCTION_MODE);
			}


			// switch from normal mode to end mode if end detected with IR6
			if(get_calibrated_prox(5) > PROX_END_DETECT && get_function_mode() == NORMAL_FUNCTION_MODE){
				set_function_mode(END_FUNCTION_MODE);
			}
		}

		chThdSleepMilliseconds(10);
	}

}

//----------------------------------------------------- EXTERNAL FUNCTIONS ------------------------------------------------------------------------------

/* start proximity detection thread */
void start_proximity_detection(void){
	chThdCreateStatic(waProximityDetection, sizeof(waProximityDetection), NORMALPRIO, ProximityDetection, NULL);
}
