#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdint.h>
#include <hal.h>

void start_regulator(void);
void set_function_mode(uint8_t mode);
uint8_t get_function_mode(void);
void rotation(uint8_t direction);
void stop_rotation(void);

#endif /* REGULATOR_H */
