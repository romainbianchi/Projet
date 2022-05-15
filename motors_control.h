#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdint.h>
#include <hal.h>

/* Start motor control thread */
void start_motors_control(void);

/* Activate clockwise or counterclockwise rotation */
void rotation(uint8_t direction);

/* Stop motors */
void stop_motors(void);

#endif /* REGULATOR_H */
