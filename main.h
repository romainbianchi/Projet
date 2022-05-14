#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define NORMAL_FUNCTION_MODE			0
#define PARABOLA_FUNCTION_MODE			1
#define ROTATION_FUNCTION_MODE			2
#define LANDING_FUNCTION_MODE			3
#define FALL_FUNCTION_MODE				4
#define CONTROL_ANGLE_FUNCTION_MODE		5

#define PI						3.1415926536f
#define WHEEL_DISTANCE      	5.35f  					// [cm]
#define WHEEL_PERIMETER			13						// [cm]
#define PERIMETER_EPUCK			(PI * WHEEL_DISTANCE)
#define NB_STEP_ONE_TURN		1000
#define INITIAL_SPEED			600 					// [steps]
#define PROX_FACTOR				0.01f
#define GOAL_PROX_VALUE			1000.00f * PROX_FACTOR
#define GOAL_OBJECT_VALUE		80u					// [mm]

#define KP 						10.0f
#define KI						0.2f
#define KD						230.0f
#define MAX_SUM_ERROR 			300
#define ERROR_THRESHOLD			1

#define SELECT_START			8
#define DEG_PER_STEP			0.141f
#define ANGLE_PARABOLA			63.3f					// [deg]
#define ANGLE_HORIZONTAL		0
#define ANGLE_THRESHOLD			1						// [deg]
#define ROTATION_SPEED			180


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
