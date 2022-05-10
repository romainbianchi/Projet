#ifndef TOF_DETECTION_H
#define TOF_DETECTION_H

#include <stdint.h>
#include <hal.h>

void start_tof_detection(void);
bool get_object_detected(void);

#endif /* TOF_DETECTION_H */
