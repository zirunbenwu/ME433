#ifndef DRV8833_H
#define DRV8833_H
#include "pico/stdlib.h"

void drv8833_init(void);
void drv8833_set_duty(float duty);   // -100..+100, slow-decay drive

#endif