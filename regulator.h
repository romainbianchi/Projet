#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdint.h>
#include <hal.h>

void start_regulator(void);
void set_function_mode(uint8_t mode);
uint8_t get_function_mode(void);

#endif /* REGULATOR_H */
