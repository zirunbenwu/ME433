#include <stdio.h>
#include "pico/stdlib.h"
 
#define BUTTON_LEFT  16
#define BUTTON_RIGHT 17
 
static void button_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}
 
int main(void) {
    stdio_init_all();
 
    button_init(BUTTON_LEFT);
    button_init(BUTTON_RIGHT);
 
    // Wait until the host opens the USB-CDC port, so the first line isn't lost.
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
 
    while (true) {
        // Active-low buttons: gpio_get == 0 means pressed, so invert.
        int left  = !gpio_get(BUTTON_LEFT);
        int right = !gpio_get(BUTTON_RIGHT);
        printf("%d,%d\n", left, right);
        sleep_ms(10);  // ~100 Hz
    }
}
