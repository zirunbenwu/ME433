#include "ina219.h"
#include "config.h"
#include "hardware/i2c.h"

#define INA219_ADDR        0x40
#define INA219_REG_CONFIG  0x00
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIB   0x05

static void ina_write16(uint8_t reg, uint16_t val) {
    uint8_t b[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    i2c_write_blocking(INA_I2C, INA219_ADDR, b, 3, false);
}

static int16_t ina_read16(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(INA_I2C, INA219_ADDR, &reg, 1, true);
    i2c_read_blocking(INA_I2C, INA219_ADDR, buf, 2, false);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void ina219_init(void) {
    i2c_init(INA_I2C, INA_FREQ);
    gpio_set_function(INA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(INA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(INA_SDA);
    gpio_pull_up(INA_SCL);
    // 10-bit, +/-160mV, cal 1024  -> current = raw/3 mA (your scaling)
    ina_write16(INA219_REG_CALIB, 1024);
    ina_write16(INA219_REG_CONFIG, 0b0011000010001111);
}

float ina219_read_ma(void) {
    return ina_read16(INA219_REG_CURRENT) / 3.0f;
}