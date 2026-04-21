#include <stdio.h>
#include "pico/stdlib.h"



int main()
{
    stdio_init_all();

    while (true) {
        //update the dac
        
        float voltageA= (sine(t+PI/2)+1)/2*3.3; //sine wave between 0 and 3.3V, phase shifted by 90 degrees
        
    }
}
