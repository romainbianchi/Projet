#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define INITIAL_SPEED			600 					// [steps]
#define PROX_FACTOR				0.01f
#define GOAL_PROX_VALUE			1000.00f * PROX_FACTOR
#define KP 						8.0f
#define KI						0.1f
#define KD						100.0f
#define MAX_SUM_ERROR 			300
#define ERROR_THRESHOLD			1
#define SELECT_START				8

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
