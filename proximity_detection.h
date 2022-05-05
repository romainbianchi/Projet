#ifndef PROXIMITY_DETECTION_H
#define PROXIMITY_DETECTION_H

#include <stdint.h>
#include <hal.h>

float conv_prox_mm(int error);
void start_proximity_detection(void);

#endif /* PROXIMITY_DETECTION_H */
