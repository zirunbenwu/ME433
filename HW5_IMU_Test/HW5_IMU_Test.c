#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define HEARTBEAT_PIN 10

// MPU6050 I2C address
#define MPU_ADDR 0x68

// MPU6050 config registers
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B

// MPU6050 sensor data registers
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I     0x75

// Write one byte to a register on the MPU6050
void mpu_write(unsigned char reg, unsigned char value) {
    unsigned char buf[2] = { reg, value };
    i2c_write_blocking(i2c_default, MPU_ADDR, buf, 2, false);
}

// Read one byte from a register on the MPU6050
unsigned char mpu_read(unsigned char reg) {
    unsigned char val;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, &val, 1, false);
    return val;
}

// Burst read 14 consecutive bytes starting at ACCEL_XOUT_H.
// Fills buf with: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L,
//                 GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
void mpu_read_burst(unsigned char *buf) {
    unsigned char reg = ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 14, false);
}

// Combine two bytes (MSB, LSB) into a signed 16-bit integer
short combine(unsigned char high, unsigned char low) {
    return (short)((high << 8) | low);
}

// Initialize the MPU6050
void mpu_init() {
    mpu_write(PWR_MGMT_1, 0x00);    // wake up the chip
    mpu_write(ACCEL_CONFIG, 0x00);  // accel: ±2g
    mpu_write(GYRO_CONFIG, 0x18);   // gyro: ±2000 dps
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

    // Wait a bit for USB serial to connect
    // (so you can open the serial monitor and not miss the first messages)
    sleep_ms(2000);

    // Init I2C0 at 400kHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Verify WHO_AM_I — accept 0x68 (datasheet) or 0x98 (variant)
    unsigned char who = mpu_read(WHO_AM_I);
    printf("WHO_AM_I = 0x%02X\n", who);
    if (who != 0x68 && who != 0x98) {
        printf("ERROR: wrong WHO_AM_I, locking up. Power cycle to reset.\n");
        gpio_put(HEARTBEAT_PIN, 1);
        while (1) { tight_loop_contents(); }
    }

    // Initialize the IMU
    mpu_init();
    printf("MPU6050 initialized. Streaming at 100Hz.\n");

    unsigned char raw[14];
    int heartbeat = 0;
    int sample_count = 0;

    while (1) {
        // Burst read all 14 bytes
        mpu_read_burst(raw);

        // Combine bytes into signed 16-bit values
        short ax_raw = combine(raw[0],  raw[1]);
        short ay_raw = combine(raw[2],  raw[3]);
        short az_raw = combine(raw[4],  raw[5]);
        short t_raw  = combine(raw[6],  raw[7]);
        short gx_raw = combine(raw[8],  raw[9]);
        short gy_raw = combine(raw[10], raw[11]);
        short gz_raw = combine(raw[12], raw[13]);

        // Convert to physical units
        float ax_g = ax_raw * 0.000061f;
        float ay_g = ay_raw * 0.000061f;
        float az_g = az_raw * 0.000061f;
        float gx_dps = gx_raw * 0.007630f;
        float gy_dps = gy_raw * 0.007630f;
        float gz_dps = gz_raw * 0.007630f;
        float temp_c = (t_raw / 340.00f) + 36.53f;

        // Print readable line
        printf("ax=%+6.3f g  ay=%+6.3f g  az=%+6.3f g  | "
               "gx=%+7.2f dps  gy=%+7.2f dps  gz=%+7.2f dps  | "
               "T=%5.2f C\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);

        // Heartbeat every 10 samples (~5Hz visible blink)
        sample_count++;
        if (sample_count % 10 == 0) {
            heartbeat = !heartbeat;
            gpio_put(HEARTBEAT_PIN, heartbeat);
        }

        // 100Hz = 10ms per sample
        sleep_ms(10);
    }

    return 0;
}