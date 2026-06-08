// bottle_sensors.c
// Combined HX711 load cell + AS5600 encoder for the haptic bartender.
// Streams one CSV line per update over USB serial:  D,<force_N>,<angle_deg>
//
// Pico (RP2040) wiring:
//   HX711:  SCK -> GP14,  DT/DOUT -> GP15,  VCC -> 3V3 (or 5V per module), GND -> GND
//   AS5600: SDA -> GP4 (I2C0),  SCL -> GP5 (I2C0),  VCC -> 3V3, GND -> GND,
//           DIR -> GND  (so clockwise increases the count, viewed from magnet side)
//           4.7k pull-ups on SDA/SCL if the module doesn't have them.

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"

// ============================ HX711 (load cell) ============================
#define PIN_SCK     14
#define PIN_DT      15
#define IIR_ALPHA   0.15f          // 1st-order LPF; ~2 Hz cutoff at fs~80 Hz

// load cell calibration:  force_N = (raw - LC_TARE) * LC_SCALE
static int32_t LC_TARE  = 0;       // raw counts at zero load (set at startup)
static float   LC_SCALE = 1.0f;    // Newtons per raw count  *** CALIBRATE ***

static float lc_filt = 0.0f;       // running filtered raw value

void hx711_init(void) {
    gpio_init(PIN_SCK);
    gpio_set_dir(PIN_SCK, GPIO_OUT);
    gpio_put(PIN_SCK, 0);
    gpio_init(PIN_DT);
    gpio_set_dir(PIN_DT, GPIO_IN);
}

int32_t hx711_read(void) {
    while (gpio_get(PIN_DT)) {
        tight_loop_contents();
    }
    uint32_t raw = 0;
    uint32_t irq = save_and_disable_interrupts();   // SCK high >60us powers down
    for (int i = 0; i < 24; i++) {
        gpio_put(PIN_SCK, 1);
        busy_wait_us(1);
        raw = (raw << 1) | gpio_get(PIN_DT);
        gpio_put(PIN_SCK, 0);
        busy_wait_us(1);
    }
    gpio_put(PIN_SCK, 1);           // 25th pulse: channel A, gain 128
    busy_wait_us(1);
    gpio_put(PIN_SCK, 0);
    busy_wait_us(1);
    restore_interrupts(irq);
    if (raw & 0x800000) raw |= 0xFF000000;          // sign-extend 24-bit
    return (int32_t)raw;
}

void hx711_tare(int n) {
    double acc = 0.0;
    for (int i = 0; i < n; i++) acc += (double)hx711_read();
    LC_TARE = (int32_t)(acc / n);
}

// ============================ AS5600 (encoder) ============================
#define I2C_PORT    i2c0
#define I2C_SDA     16
#define I2C_SCL     17
#define I2C_FREQ    400000

#define AS5600_ADDR     0x36
#define REG_STATUS      0x0B
#define REG_ANGLE       0x0E       // filtered 12-bit angle (0x0E..0x0F)
#define REG_AGC         0x1A
#define STATUS_MH   (1 << 3)
#define STATUS_ML   (1 << 4)
#define STATUS_MD   (1 << 5)

static uint16_t enc_zero_raw = 0;  // startup position captured as zero
static int      enc_dir_sign = +1; // +1 if CW positive; set -1 to flip

bool as5600_read_reg(uint8_t reg, uint8_t *value) {
    int w = i2c_write_blocking(I2C_PORT, AS5600_ADDR, &reg, 1, true);
    if (w != 1) return false;
    int r = i2c_read_blocking(I2C_PORT, AS5600_ADDR, value, 1, false);
    return (r == 1);
}

bool as5600_read_12bit(uint8_t reg_high, uint16_t *value) {
    uint8_t buf[2];
    int w = i2c_write_blocking(I2C_PORT, AS5600_ADDR, &reg_high, 1, true);
    if (w != 1) return false;
    int r = i2c_read_blocking(I2C_PORT, AS5600_ADDR, buf, 2, false);
    if (r != 2) return false;
    *value = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];   // mask high nibble
    return true;
}

bool as5600_present(void) {
    uint8_t dummy;
    int r = i2c_read_blocking(I2C_PORT, AS5600_ADDR, &dummy, 1, false);
    return (r >= 0);
}

bool as5600_set_zero(void) {
    uint16_t raw;
    if (!as5600_read_12bit(REG_ANGLE, &raw)) return false;
    enc_zero_raw = raw;
    return true;
}

// Tared, signed angle in degrees, CW positive, wrapped to (-180, +180].
bool as5600_get_angle_deg(float *deg) {
    uint16_t raw;
    if (!as5600_read_12bit(REG_ANGLE, &raw)) return false;
    int diff = (int)raw - (int)enc_zero_raw;
    if (diff > 2048)   diff -= 4096;
    if (diff <= -2048) diff += 4096;
    *deg = enc_dir_sign * diff * (360.0f / 4096.0f);
    return true;
}

// ============================ main ============================
int main(void) {
    stdio_init_all();
    sleep_ms(1500);                // let USB serial enumerate

    // --- init HX711 ---
    hx711_init();
    (void)hx711_read();            // first reading uses wrong gain, discard

    // --- init AS5600 I2C ---
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // startup info (the game ignores any line not starting with "D,")
    printf("# bottle sensors starting\n");
    if (as5600_present()) {
        printf("# AS5600 found at 0x%02X\n", AS5600_ADDR);
        uint8_t status;
        if (as5600_read_reg(REG_STATUS, &status)) {
            printf("# encoder STATUS=0x%02X %s%s%s\n", status,
                   (status & STATUS_MD) ? "magnet-OK" : "NO-magnet",
                   (status & STATUS_ML) ? " too-weak" : "",
                   (status & STATUS_MH) ? " too-strong" : "");
        }
    } else {
        printf("# WARNING: AS5600 not responding\n");
    }

    // --- tare both sensors (keep handle unloaded + still at startup) ---
    sleep_ms(300);
    hx711_tare(20);
    lc_filt = (float)LC_TARE;       // seed filter so force starts near 0
    printf("# load cell tare = %ld\n", (long)LC_TARE);

    if (as5600_set_zero())
        printf("# encoder zero set (raw %u)\n", enc_zero_raw);
    else
        printf("# WARNING: encoder zero not set\n");

    printf("# streaming: D,force_N,angle_deg\n");

    // ---- continuous stream ----
    // The HX711 read blocks until its next sample (~80 Hz), pacing the loop.
    // After each load cell sample we read the encoder (fast over I2C) and emit.
    while (true) {
        int32_t r = hx711_read();
        lc_filt = IIR_ALPHA * (float)r + (1.0f - IIR_ALPHA) * lc_filt;
        float force_N = (lc_filt - (float)LC_TARE) * LC_SCALE;

        float angle_deg = 0.0f;
        as5600_get_angle_deg(&angle_deg);   // if read fails, keeps last/0

        // single data line the game parses:  D,force,angle
        printf("D,%.3f,%.2f\n", force_N, angle_deg);
    }
    return 0;
}