#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include <sensors/imu.h>

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define INITIAL_SPEED			700 	// [steps]
//#define GRAVITY					9.81 	// [m/s^2]

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void show_gravity(imu_msg_t imu_values);

#ifdef __cplusplus
}
#endif

#endif
