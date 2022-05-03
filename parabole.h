#ifndef PARABOLE_H
#define PARABOLE_H

#define PI 3.14

void start_parabola(void);
float speed_conversion_cm_to_step(float cm_speed);
float calculate_outer_speed(float roc);
float calculate_inner_speed(float roc);
float calculate_roc(float t);
#endif
