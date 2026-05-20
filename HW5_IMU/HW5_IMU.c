#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define HEARTBEAT_PIN 10

// MPU6050 I2C address
#define MPU_ADDR 0x68

// MPU6050 config registers
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

// MPU6050 sensor data registers
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I     0x75

// OLED dimensions (128x32)
#define SCREEN_W 128
#define SCREEN_H 32
#define CENTER_X 64
#define CENTER_Y 16

// ---------- I2C helpers ----------

void mpu_write(unsigned char reg, unsigned char value) {
    unsigned char buf[2] = { reg, value };
    i2c_write_blocking(i2c_default, MPU_ADDR, buf, 2, false);
}

unsigned char mpu_read(unsigned char reg) {
    unsigned char val;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, &val, 1, false);
    return val;
}

// Burst read of 14 consecutive bytes starting at ACCEL_XOUT_H.
// Fills buf with: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L,
//                 GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L.
void mpu_read_burst(unsigned char *buf) {
    unsigned char reg = ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 14, false);
}

// ---------- MPU6050 init ----------

void mpu_init() {
    // Wake the chip up
    mpu_write(PWR_MGMT_1, 0x00);
    // Accelerometer: ±2g (bits 4:3 = 00)
    mpu_write(ACCEL_CONFIG, 0x00);
    // Gyroscope: ±2000 dps (bits 4:3 = 11)
    mpu_write(GYRO_CONFIG, 0x18);
}

// Combine two bytes into a signed 16-bit integer (big-endian, MSB first)
short combine(unsigned char high, unsigned char low) {
    return (short)((high << 8) | low);
}

// ---------- Font / drawing helpers ----------

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

void drawMessage(unsigned char x, unsigned char y, char *m) {
    while (*m != 0) {
        drawChar(x, y, *m);
        x += 5;
        m++;
    }
}

// Draw a line from (x0,y0) to (x1,y1) using Bresenham's algorithm.
// Works for any direction.
void drawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        if (x0 >= 0 && x0 < SCREEN_W && y0 >= 0 && y0 < SCREEN_H) {
            ssd1306_drawPixel(x0, y0, 1);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// ---------- main ----------

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

    // Init OLED
    ssd1306_setup();
    ssd1306_clear();
    drawMessage(0, 0, "Checking IMU...");
    ssd1306_update();
    sleep_ms(1000);   

    // Verify WHO_AM_I — accept 0x68 (datasheet) or 0x98 (variant chips)
    unsigned char who = mpu_read(WHO_AM_I);
    if (who != 0x68 && who != 0x98) {
        // Wrong chip / wiring problem — show error and lock up with LED on
        ssd1306_clear();
        char msg[40];
        sprintf(msg, "IMU FAIL who=0x%02X", who);
        drawMessage(0, 0, msg);
        drawMessage(0, 12, "Power cycle me!");
        ssd1306_update();
        gpio_put(HEARTBEAT_PIN, 1);
        while (1) { tight_loop_contents(); }
    }

    // Initialize the IMU
    mpu_init();

    unsigned char raw[14];
    int heartbeat = 0;
    int frame_count = 0;

    while (1) {
        // Read all 14 bytes in one burst
        mpu_read_burst(raw);

        // Combine into signed 16-bit values
        short ax_raw = combine(raw[0],  raw[1]);
        short ay_raw = combine(raw[2],  raw[3]);


        // Convert to g's
        float ax_g = ax_raw * 0.000061f;
        float ay_g = ay_raw * 0.000061f;


        float scale = 14.0f;
        int line_x = (int)(ax_g * scale);
        int line_y = (int)(ay_g * scale);

        // Clear and redraw
        ssd1306_clear();


        drawLine(CENTER_X, CENTER_Y, CENTER_X - line_x, CENTER_Y + line_y);

        // Also draw a small text readout
        char msg[32];
        sprintf(msg, "x%+.2f y%+.2f", ax_g, ay_g);
        drawMessage(0, 24, msg);

        ssd1306_update();

        // Heartbeat: toggle every ~30 frames so it's visible at high fps
        frame_count++;
        if (frame_count % 15 == 0) {
            heartbeat = !heartbeat;
            gpio_put(HEARTBEAT_PIN, heartbeat);
        }
    }

    return 0;
}
