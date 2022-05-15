#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/* function modes */
#define NORMAL_MODE						0
#define PARABOLA_MODE					1
#define ROTATION_MODE					2
#define INV_ROTATION_MODE				3
#define LANDING_MODE					4
#define INV_LANDING_MODE				5
#define FALL_MODE						6
#define CONTROL_PARA_ANGLE_MODE			7
#define CONTROL_HORZ_ANGLE_MODE			8
#define END_MODE						9

/* start when the selector is in position 8 */
#define SELECT_START					8

/* get the actual mode */
uint8_t get_function_mode(void);

/* Set the mode */
void set_function_mode(uint8_t mode);

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
