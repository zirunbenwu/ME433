#ifndef AS5600_H
#define AS5600_H
#include "pico/stdlib.h"

void  as5600_init(void);
void  as5600_set_zero(void);
float as5600_angle_deg(void);    // tared, signed, wrapped (-180,180]

#endif