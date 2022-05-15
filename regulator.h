#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdint.h>
#include <hal.h>

void start_regulator(void);
void rotation(uint8_t direction);
void stop_motors(void);

#endif /* REGULATOR_H */
