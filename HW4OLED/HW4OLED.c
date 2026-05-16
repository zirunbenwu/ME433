#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "ssd1306.h"
#include "font.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define HEARTBEAT_PIN 10

#define ADC0_PIN 26   // GP26 = ADC channel 0

// Draw a single character at pixel position (x, y).
// Characters are 5 pixels wide and 8 pixels tall.
void drawChar(unsigned char x, unsigned char y, char c) {
    if (c < 0x20 || c > 0x7F) return;
    int index = c - 0x20;

    for (int col = 0; col < 5; col++) {
        char column_data = ASCII[index][col];
        for (int row = 0; row < 8; row++) {
            char pixel_on = (column_data >> row) & 0x01;
            ssd1306_drawPixel(x + col, y + row, pixel_on);
        }
    }
}

// Draw a null-terminated character array starting at (x, y).
void drawMessage(unsigned char x, unsigned char y, char *m) {
    while (*m != 0) {
        drawChar(x, y, *m);
        x += 5;
        m++;
    }
}

int main() {
    stdio_init_all();

    // Heartbeat LED
    gpio_init(HEARTBEAT_PIN);
    gpio_set_dir(HEARTBEAT_PIN, GPIO_OUT);

    // Startup blink
    for (int i = 0; i < 5; i++) {
        gpio_put(HEARTBEAT_PIN, 1); sleep_ms(150);
        gpio_put(HEARTBEAT_PIN, 0); sleep_ms(150);
    }
    sleep_ms(500);

    // Init I2C0 at 400kHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Init ADC on GP26 (ADC0)
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_select_input(0);

    // Init OLED
    ssd1306_setup();

    char message[50];
    int heartbeat = 0;
    int frame_count = 0;

    while (1) {
        // Mark start time of this frame
        unsigned int t_start = to_us_since_boot(get_absolute_time());

        // Read ADC0 — 12-bit value, 0..4095, ref voltage 3.3V
        unsigned short raw = adc_read();
        float volts = raw * 3.3f / 4095.0f;

        // Clear and redraw
        ssd1306_clear();

        sprintf(message, "ADC0 = %.3f V", volts);
        drawMessage(0, 0, message);

        // Frame count just so we can see it's actively updating
        sprintf(message, "frame %d", frame_count++);
        drawMessage(0, 12, message);

        // Push to display — this is the slowest part
        ssd1306_update();

        // Mark end time, compute fps from elapsed microseconds
        unsigned int t_end = to_us_since_boot(get_absolute_time());
        unsigned int elapsed_us = t_end - t_start;
        float fps = 1000000.0f / (float)elapsed_us;

        // Draw fps on the bottom row — note this won't be pushed to the
        // display until the NEXT ssd1306_update() call, so the fps shown
        // is from the previous frame. Close enough for monitoring.
        sprintf(message, "fps = %.1f", fps);
        drawMessage(0, 24, message);

        // Toggle heartbeat (no sleep — we want max fps)
        heartbeat = !heartbeat;
        gpio_put(HEARTBEAT_PIN, heartbeat);
    }

    return 0;
}