#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define NORMAL_FUNCTION_MODE					0
#define PARABOLA_FUNCTION_MODE					1
#define ROTATION_FUNCTION_MODE					2
#define INV_ROTATION_FUNCTION_MODE				3
#define LANDING_FUNCTION_MODE					4
#define FALL_FUNCTION_MODE						5
#define CONTROL__PARA ANGLE_FUNCTION_MODE		6
#define END_FUNCTION_MODE						7

#define SELECT_START							8

uint8_t get_function_mode(void);
void set_function_mode(uint8_t mode);

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
