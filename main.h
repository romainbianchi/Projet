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
#define INITIAL_SPEED			700 	// [steps]
#define GRAVITY					9.81 	// [m/s^2]
#define GOAL_PROX_VALUE			150		// Proximity sensor value
#define KP						1000.0f
#define KI						3.5f
#define CORRECTION_COEFF		0.01
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
