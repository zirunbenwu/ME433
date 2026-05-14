#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MCP23008 I2C address with A0=A1=A2=GND
#define ADDR 0x20

// MCP23008 register addresses
#define IODIR  0x00  // I/O direction: 1=input, 0=output
#define GPIO   0x09  // read pin states
#define OLAT   0x0A  // write pin states

// Pico pins for I2C0
#define SDA_PIN 12
#define SCL_PIN 13

// External heartbeat LED — 
#define HEARTBEAT_PIN 10

// Write a value to a register on the MCP23008
void setPin(unsigned char address, unsigned char reg, unsigned char value) {
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(i2c_default, address, buf, 2, false);
}

// Read a value from a register on the MCP23008
unsigned char readPin(unsigned char address, unsigned char reg) {
    unsigned char buf;
    i2c_write_blocking(i2c_default, address, &reg, 1, true);   // keep bus
    i2c_read_blocking(i2c_default, address, &buf, 1, false);   // release bus
    return buf;
}

int main() {
    stdio_init_all();

    // Init heartbeat LED
    gpio_init(HEARTBEAT_PIN);
    gpio_set_dir(HEARTBEAT_PIN, GPIO_OUT);

    // Startup blink — 5 fast blinks proves the Pico is running
    for (int i = 0; i < 5; i++) {
        gpio_put(HEARTBEAT_PIN, 1);
        sleep_ms(150);
        gpio_put(HEARTBEAT_PIN, 0);
        sleep_ms(150);
    }
    sleep_ms(500);

    // Init I2C0 at 100kHz
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Configure MCP23008:
    // GP7 = output (bit 7 = 0), all others = input (bits 6:0 = 1)
    setPin(ADDR, IODIR, 0x7F);

    // Start with LED off
    setPin(ADDR, OLAT, 0x00);

    unsigned char gpio_state;
    int heartbeat = 0;
    int count = 0;

    while (1) {
        // Read all 8 pins from GPIO register
        gpio_state = readPin(ADDR, GPIO);

        // Check GP0 (bit 0): button has external pull-up,
        // so pressed = 0, released = 1
        if (!(gpio_state & 0x01)) {
            // Button pressed: turn on GP7
            setPin(ADDR, OLAT, (1 << 7));
        } else {
            // Button released: turn off GP7
            setPin(ADDR, OLAT, 0x00);
        }

        // Heartbeat: toggle Pico LED every ~10 loops (~500ms)
        count++;
        if (count >= 10) {
            count = 0;
            heartbeat = !heartbeat;
            gpio_put(HEARTBEAT_PIN, heartbeat);
        }

        sleep_ms(50);
    }

    return 0;
}