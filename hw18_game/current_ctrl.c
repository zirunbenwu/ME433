#include "current_ctrl.h"
#include "config.h"
#include "drv8833.h"
#include "ina219.h"
#include <math.h>

volatile enum mode_t state = IDLE;
volatile float Kp_current = 0.0f;
volatile float Ki_current = 0.0f;
volatile float desired_current = 0.0f;
volatile float duty_cycle = 0.0f;
volatile float BUZZ_AMP_MA  = 120.0f;
volatile float BUZZ_FREQ_HZ = 16.0f;
volatile uint32_t buzz_until_ms = 0;
volatile float g_current_ma = 0.0f;

volatile float itest_desired[ITEST_SAMPLES];
volatile float itest_actual[ITEST_SAMPLES];
static volatile int itest_index = 0;

static volatile float eint_current = 0.0f;
static volatile float buzz_phase   = 0.0f;

static float current_PI(float des_mA, float act_mA) {
    float error = des_mA - act_mA;
    eint_current += error;
    if (eint_current >  EINT_MAX) eint_current =  EINT_MAX;
    if (eint_current < -EINT_MAX) eint_current = -EINT_MAX;
    float u = Kp_current * error + Ki_current * eint_current;
    if (u >  100.0f) u =  100.0f;
    if (u < -100.0f) u = -100.0f;
    return u;
}

static bool current_control_cb(struct repeating_timer *t) {
    switch (state) {
        case IDLE:
            drv8833_set_duty(0.0f);
            break;

        case PWM_MODE:
            drv8833_set_duty(duty_cycle);
            break;

        case ITEST: {
            int segment = itest_index / ITEST_SEGMENT;
            if (itest_index % ITEST_SEGMENT == 0) eint_current = 0.0f;
            float des = (segment % 2 == 0) ? -ITEST_CURRENT : +ITEST_CURRENT;
            float act = ina219_read_ma();          // <-- read fresh here (was g_current_ma)
            drv8833_set_duty(current_PI(des, act));
            itest_desired[itest_index] = des;
            itest_actual[itest_index]  = act;
            itest_index++;
            if (itest_index >= ITEST_SAMPLES) {
                itest_index = 0; eint_current = 0.0f; state = IDLE;
            }
            break;
        }

        case BUZZ: {
            if (to_ms_since_boot(get_absolute_time()) > buzz_until_ms) {
                state = IDLE; desired_current = 0.0f; drv8833_set_duty(0.0f);
                break;
            }
            buzz_phase += 2.0f * (float)M_PI * BUZZ_FREQ_HZ * 0.001f;
            if (buzz_phase > 2.0f * (float)M_PI) buzz_phase -= 2.0f * (float)M_PI;
            float des = BUZZ_AMP_MA * sinf(buzz_phase);
            if (des >  DES_CUR_LIMIT_MA) des =  DES_CUR_LIMIT_MA;
            if (des < -DES_CUR_LIMIT_MA) des = -DES_CUR_LIMIT_MA;
            desired_current = des;
            drv8833_set_duty(current_PI(des, g_current_ma));
            break;
        }
    }
    return true;
}

void current_ctrl_reset(void) {
    eint_current = 0.0f;
    buzz_phase   = 0.0f;
    itest_index  = 0;
}

void current_ctrl_start(struct repeating_timer *timer) {
    add_repeating_timer_us(-1000, current_control_cb, NULL, timer);
}