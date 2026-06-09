#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"

// ============================ HX711 (load cell) ============================
#define PIN_SCK         16
#define PIN_DT          17

#define MAX_SAMPLES     4000
#define IIR_ALPHA       0.15f  // 1st-order LPF; ~2 Hz cutoff at fs=80Hz

static int32_t  raw_buf[MAX_SAMPLES];
static float    filt_buf[MAX_SAMPLES];
static uint32_t time_buf[MAX_SAMPLES];
static float    ang_buf[MAX_SAMPLES];     // encoder angle per sample

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

    // SCK held HIGH > ~60 us powers down the HX711, so block IRQs.
    uint32_t irq = save_and_disable_interrupts();

    for (int i = 0; i < 24; i++) {
        gpio_put(PIN_SCK, 1);
        busy_wait_us(1);
        raw = (raw << 1) | gpio_get(PIN_DT);
        gpio_put(PIN_SCK, 0);
        busy_wait_us(1);
    }

    // 25th pulse: select channel A, gain 128 for next conversion.
    gpio_put(PIN_SCK, 1);
    busy_wait_us(1);
    gpio_put(PIN_SCK, 0);
    busy_wait_us(1);

    restore_interrupts(irq);

    // sign-extend 24-bit two's complement to 32-bit signed int
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
    return (int32_t)raw;
}

// ============================ AS5600 (encoder) ============================
#define I2C_PORT    i2c0
#define I2C_SDA     12
#define I2C_SCL     13
#define I2C_FREQ    400000

#define AS5600_ADDR     0x36
#define REG_STATUS      0x0B
#define REG_ANGLE       0x0E       // filtered 12-bit angle (0x0E..0x0F)
#define STATUS_MD   (1 << 5)

static uint16_t enc_zero_raw = 0;  // startup position captured as zero
static int      enc_dir_sign = -1; // +1 if CW positive; set -1 to flip

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
// Returns 0.0 on read failure.
float as5600_angle_deg(void) {
    uint16_t raw;
    if (!as5600_read_12bit(REG_ANGLE, &raw)) return 0.0f;
    int diff = (int)raw - (int)enc_zero_raw;
    if (diff > 2048)   diff -= 4096;
    if (diff <= -2048) diff += 4096;
    return enc_dir_sign * diff * (360.0f / 4096.0f);
}

// ============================ main ============================
int main(void) {
    stdio_init_all();
    hx711_init();

    // --- init AS5600 I2C ---
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // first reading uses wrong gain/channel, discard
    (void)hx711_read();

    // capture encoder startup position as zero
    as5600_set_zero();

    while (true) {
        int n = 0;
        int ret = scanf("%d", &n);
        if (ret != 1) {
            // bad/non-digit input - consume one char to avoid spinning
            getchar_timeout_us(1000);
            continue;
        }
        if (n < 0) continue;
        if (n > MAX_SAMPLES) n = MAX_SAMPLES;

        if (n == 0) {
            // streaming mode: print samples until any byte arrives on stdin
            int32_t first = hx711_read();
            float   filt  = (float)first;
            float   ang   = as5600_angle_deg();
            absolute_time_t t_start = get_absolute_time();

            printf("STREAM\n");
            printf("%ld,%.3f,0,%.2f\n", (long)first, filt, ang);

            while (true) {
                if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                    printf("STOP\n");
                    break;
                }
                int32_t r = hx711_read();
                filt = IIR_ALPHA * (float)r + (1.0f - IIR_ALPHA) * filt;
                ang  = as5600_angle_deg();
                uint32_t t_ms = to_ms_since_boot(get_absolute_time())
                              - to_ms_since_boot(t_start);
                printf("%ld,%.3f,%lu,%.2f\n",
                       (long)r, filt, (unsigned long)t_ms, ang);
            }
            continue;
        }

        // batch mode: capture N samples then dump them
        int32_t first = hx711_read();
        float   filt  = (float)first;
        absolute_time_t t_start = get_absolute_time();

        raw_buf[0]  = first;
        filt_buf[0] = filt;
        time_buf[0] = 0;
        ang_buf[0]  = as5600_angle_deg();

        for (int i = 1; i < n; i++) {
            int32_t r = hx711_read();
            filt = IIR_ALPHA * (float)r + (1.0f - IIR_ALPHA) * filt;

            raw_buf[i]  = r;
            filt_buf[i] = filt;
            time_buf[i] = to_ms_since_boot(get_absolute_time())
                        - to_ms_since_boot(t_start);
            ang_buf[i]  = as5600_angle_deg();
        }

        printf("BEGIN %d\n", n);
        for (int i = 0; i < n; i++) {
            printf("%ld,%.3f,%lu,%.2f\n",
                   (long)raw_buf[i], filt_buf[i],
                   (unsigned long)time_buf[i], ang_buf[i]);
        }
        printf("END\n");
    }
}
