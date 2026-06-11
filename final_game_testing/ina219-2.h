// function prototypes for the INA219 current sensor
 
#ifndef INA219_H
#define INA219_H
 
#include "pico/stdlib.h"
#include "hardware/i2c.h"
 
void init_ina219();
float read_ina219();
 
#endif