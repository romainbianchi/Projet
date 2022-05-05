#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define INITIAL_SPEED			600 					// [steps]
#define GRAVITY					9.81 					// [m/s^2]
#define PROX_FACTOR				0.01f
#define GOAL_PROX_VALUE			1000.00f * PROX_FACTOR
#define KP 						10.0f
#define KI						0.5f
#define KD						500.0f
#define MAX_SUM_ERROR 			200
#define ERROR_THRESHOLD			3
#define PROX_MM_FACTOR			0.0165

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
