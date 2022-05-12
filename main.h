#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define NORMAL_FUNCTION_MODE	0
#define PARABOLA_FUNCTION_MODE	1
#define FALL_FUNCTION_MODE		2
#define PI						3.1415926536f
#define WHEEL_DISTANCE      	5.35f  					// [cm]
#define WHEEL_PERIMETER			13						// [cm]
#define PERIMETER_EPUCK			(PI * WHEEL_DISTANCE)
#define NB_STEP_ONE_TURN		1000
#define INITIAL_SPEED			600 					// [steps]
#define PROX_FACTOR				0.01f
#define GOAL_PROX_VALUE			1000.00f * PROX_FACTOR
#define GOAL_TOF_VALUE			120						// [mm]
#define KP 						8.0f
#define KI						0.1f
#define KD						100.0f
#define MAX_SUM_ERROR 			300
#define ERROR_THRESHOLD			1
#define SELECT_START			8
#define DEG_PER_STEP			0.141f
#define ANGLE_AT_NINETY			919.91f

static uint8_t function_mode = NORMAL_FUNCTION_MODE;

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
