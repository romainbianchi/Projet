#ifndef PROXIMITY_DETECTION_H
#define PROXIMITY_DETECTION_H

#include <stdint.h>
#include <hal.h>

int get_distance(void);
void start_proximity_detection(void);
void start_proximity_motors(void);

#endif /* PROXIMITY_DETECTION_H */
