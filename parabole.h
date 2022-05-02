#ifndef PARABOLE_H
#define PARABOLE_H

#define PI 3.14

void start_circling(void);
float speed_conversion_cm_to_step(uint16_t cm_speed);
float outer_speed(float inner_speed, float d);

#endif
