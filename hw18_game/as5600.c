#include "as5600.h"
#include "config.h"
#include "hardware/i2c.h"

#define AS5600_ADDR    0x36
#define AS5600_ANGLE   0x0E      // filtered 12-bit angle (0x0E..0x0F)

static uint16_t as_zero_raw = 0;
static int      as_dir_sign = -1;     // set +1 to flip CW/CCW
static float    last_deg    = 0.0f;

static bool as_read12(uint8_t reg, uint16_t *v) {
    uint8_t buf[2];
    if (i2c_write_blocking(AS_I2C, AS5600_ADDR, &reg, 1, true) != 1) return false;
    if (i2c_read_blocking(AS_I2C, AS5600_ADDR, buf, 2, false) != 2) return false;
    *v = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];   // mask high nibble
    return true;
}

void as5600_init(void) {
    i2c_init(AS_I2C, AS_FREQ);
    gpio_set_function(AS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(AS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(AS_SDA);
    gpio_pull_up(AS_SCL);
}

void as5600_set_zero(void) {
    as_read12(AS5600_ANGLE, &as_zero_raw);
}

float as5600_angle_deg(void) {
    uint16_t raw;
    if (!as_read12(AS5600_ANGLE, &raw)) return last_deg;   // keep last on fail
    int diff = (int)raw - (int)as_zero_raw;
    if (diff > 2048)   diff -= 4096;
    if (diff <= -2048) diff += 4096;
    last_deg = as_dir_sign * diff * (360.0f / 4096.0f);
    return last_deg;
}