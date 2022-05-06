#ifndef PROXIMITY_DETECTION_H
#define PROXIMITY_DETECTION_H

#include <stdint.h>
#include <hal.h>

void start_proximity_detection(void);
bool get_object_detected(void);

#endif /* PROXIMITY_DETECTION_H */
