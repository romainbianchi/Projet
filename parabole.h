#ifndef PARABOLE_H
#define PARABOLE_H

#define PI 3.14

void start_parabola(void);
float speed_conversion_cm_to_step(float cm_speed);
float calculate_outer_speed(float roc, float v);
float calculate_inner_speed(float roc, float v);
float calculate_roc(float t);
float calculate_norm_speed(float t);

float x(float t);
float d1d_x(float t);
float d2d_x(float t);
float y(float t);
float d1d_y(float t);
float d2d_y(float t);

float d_roc(float t);
float d_norm_speed(float t);
float d_angle(float t);

#endif
