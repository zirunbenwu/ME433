#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"

#define SDA_PIN 12
#define SCL_PIN 13
#define HEARTBEAT_PIN 15

int main() {
    stdio_init_all();

    // Heartbeat LED setup
    gpio_init(HEARTBEAT_PIN);
    gpio_set_dir(HEARTBEAT_PIN, GPIO_OUT);

    // Startup blink — proves Pico is running before any I2C
    for (int i = 0; i < 5; i++) {
        gpio_put(HEARTBEAT_PIN, 1);
        sleep_ms(150);
        gpio_put(HEARTBEAT_PIN, 0);
        sleep_ms(150);
    }
    sleep_ms(500);

    // Init I2C0 at 400kHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Init the SSD1306 OLED
    ssd1306_setup();

    int block_on = 0;
    int heartbeat = 0;

    while (1) {
        // Toggle an 8x8 block centered around (64, 16) on the 128x32 display
        block_on = !block_on;
        for (int dx = 0; dx < 8; dx++) {
            for (int dy = 0; dy < 8; dy++) {
                ssd1306_drawPixel(60 + dx, 12 + dy, block_on);
            }
        }
        ssd1306_update();

        // Toggle heartbeat LED in sync (1Hz blink: 500ms on, 500ms off)
        heartbeat = !heartbeat;
        gpio_put(HEARTBEAT_PIN, heartbeat);

        sleep_ms(500);
    }

    return 0;
}
