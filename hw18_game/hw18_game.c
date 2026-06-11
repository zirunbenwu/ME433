#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "config.h"
#include "drv8833.h"
#include "ina219.h"
#include "as5600.h"
#include "current_ctrl.h"

static char  rxbuf[48];
static int   rxlen = 0;
static float g_angle_deg = 0.0f;

// ---------------------------------------------------------------
// Serial command set
//   tuner:  a f<duty> g<kp><ki> h k y z p q
//   bench:  b  v<amp><freq>
//   game:   B<amp><dur>  S
// ---------------------------------------------------------------
static void handle_line(char *s) {
    switch (s[0]) {
        case 'a':
            printf("%.2f\n", g_current_ma);   // cached current
            break;
        case 'f': {
            float d = 0; sscanf(s + 1, "%f", &d);
            duty_cycle = d; state = PWM_MODE;
            break;
        }
        case 'g': {
            float kp = 0, ki = 0; sscanf(s + 1, "%f %f", &kp, &ki);
            Kp_current = kp; Ki_current = ki;
            printf("# gains Kp=%.4f Ki=%.4f\n", Kp_current, Ki_current);
            break;
        }
        case 'h':
            printf("%.4f %.4f\n", Kp_current, Ki_current);
            break;
        case 'k': {
            current_ctrl_reset(); state = ITEST;
            while (state == ITEST) tight_loop_contents();
            printf("# ITEST %d\n", ITEST_SAMPLES);
            for (int i = 0; i < ITEST_SAMPLES; i++)
                printf("%.2f %.2f\n", itest_desired[i], itest_actual[i]);
            break;
        }
        case 'y':
            printf("%.2f\n", g_angle_deg);
            break;
        case 'z':
            as5600_set_zero();
            break;
        case 'B': {                       // buzz burst from the game
            float amp = 0, dur = 0; sscanf(s + 1, "%f %f", &amp, &dur);
            if (amp > 0) BUZZ_AMP_MA = amp;
            buzz_until_ms = to_ms_since_boot(get_absolute_time()) + (uint32_t)dur;
            if (state != BUZZ) { current_ctrl_reset(); state = BUZZ; }
            break;
        }
        case 'v': {                       // set buzz params: v <amp> <freq>
            float a = 0, fz = 0; sscanf(s + 1, "%f %f", &a, &fz);
            if (a > 0)  BUZZ_AMP_MA  = a;
            if (fz > 0) BUZZ_FREQ_HZ = fz;
            printf("# buzz amp=%.0f f=%.1f\n", BUZZ_AMP_MA, BUZZ_FREQ_HZ);
            break;
        }
        case 'b': {                       // manual continuous buzz (bench)
            current_ctrl_reset();
            buzz_until_ms = to_ms_since_boot(get_absolute_time()) + 600000; // 10 min
            state = BUZZ;
            printf("# BUZZ amp=%.0f f=%.1f\n", BUZZ_AMP_MA, BUZZ_FREQ_HZ);
            break;
        }
        case 'S':
        case 'p':
        case 'q':
            state = IDLE;
            desired_current = 0.0f;
            buzz_until_ms = 0;            // force any buzz to be expired
            current_ctrl_reset();
            drv8833_set_duty(0.0f);       // kill motor output immediately
            break;
        default:
            break;
    }
}

static void poll_serial(void) {
    int ch = getchar_timeout_us(0);
    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            rxbuf[rxlen] = '\0';
            if (rxlen > 0) handle_line(rxbuf);
            rxlen = 0;
        } else if (rxlen < (int)sizeof(rxbuf) - 1) {
            rxbuf[rxlen++] = (char)ch;
        } else {
            rxlen = 0;   // overflow guard
        }
        ch = getchar_timeout_us(0);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    ina219_init();
    drv8833_init();
    as5600_init();

    drv8833_set_duty(0.0f);
    as5600_set_zero();
    state = IDLE;

    struct repeating_timer current_timer;
    current_ctrl_start(&current_timer);   // 1 kHz control loop

    printf("# READY\n");
    printf("# tuner: a f<duty> g<kp><ki> h k y z p q | bench: b v<amp><freq> | game: B<amp><dur> S\n");
    printf("# streaming: angle,current_mA\n");

    while (true) {
        poll_serial();
        g_angle_deg  = as5600_angle_deg();
        g_current_ma = ina219_read_ma();   // I2C read here in main loop (safe), cached for ISR
        // don't stream during ITEST so the dump stays clean
        if (state != ITEST) {
            printf("%.2f,%.2f\n", g_angle_deg, g_current_ma);
        }
        sleep_ms(5);
    }
}