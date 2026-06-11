#ifndef INA219_H
#define INA219_H
#include "pico/stdlib.h"

void  ina219_init(void);
float ina219_read_ma(void);     // signed current, mA

#endif