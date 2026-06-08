#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
 
#define PIN_SCK         17
#define PIN_DT          16
 
#define MAX_SAMPLES     4000
#define IIR_ALPHA       0.15f
 
static int32_t  raw_buf[MAX_SAMPLES];
static float    filt_buf[MAX_SAMPLES];
static uint32_t time_buf[MAX_SAMPLES];
 
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
 
int main(void) {
    stdio_init_all();
    hx711_init();
 
    // first reading uses wrong gain/channel, discard
    (void)hx711_read();
 
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
            absolute_time_t t_start = get_absolute_time();
 
            printf("STREAM\n");
            printf("%ld,%.3f,0\n", (long)first, filt);
 
            while (true) {
                if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                    printf("STOP\n");
                    break;
                }
                int32_t r = hx711_read();
                filt = IIR_ALPHA * (float)r + (1.0f - IIR_ALPHA) * filt;
                uint32_t t_ms = to_ms_since_boot(get_absolute_time())
                              - to_ms_since_boot(t_start);
                printf("%ld,%.3f,%lu\n",
                       (long)r, filt, (unsigned long)t_ms);
            }
            continue;
        }
 
        // batch capture mode
        int32_t first = hx711_read();
        float   filt  = (float)first;
        absolute_time_t t_start = get_absolute_time();
 
        raw_buf[0]  = first;
        filt_buf[0] = filt;
        time_buf[0] = 0;
 
        for (int i = 1; i < n; i++) {
            int32_t r = hx711_read();
            filt = IIR_ALPHA * (float)r + (1.0f - IIR_ALPHA) * filt;
 
            raw_buf[i]  = r;
            filt_buf[i] = filt;
            time_buf[i] = to_ms_since_boot(get_absolute_time())
                        - to_ms_since_boot(t_start);
        }
 
        printf("BEGIN %d\n", n);
        for (int i = 0; i < n; i++) {
            printf("%ld,%.3f,%lu\n",
                   (long)raw_buf[i], filt_buf[i], (unsigned long)time_buf[i]);
        }
        printf("END\n");
    }
}
