#include "drv8833.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

static uint in1_slice, in1_chan, in2_slice, in2_chan;
static uint32_t pwm_wrap;

void drv8833_init(void) {
    pwm_wrap = (clock_get_hz(clk_sys) / PWM_FREQ_HZ) - 1;

    gpio_set_function(IN1_PIN, GPIO_FUNC_PWM);
    in1_slice = pwm_gpio_to_slice_num(IN1_PIN);
    in1_chan  = pwm_gpio_to_channel(IN1_PIN);
    pwm_set_wrap(in1_slice, pwm_wrap);
    pwm_set_clkdiv(in1_slice, 1.0f);
    pwm_set_chan_level(in1_slice, in1_chan, pwm_wrap + 1);

    gpio_set_function(IN2_PIN, GPIO_FUNC_PWM);
    in2_slice = pwm_gpio_to_slice_num(IN2_PIN);
    in2_chan  = pwm_gpio_to_channel(IN2_PIN);
    pwm_set_chan_level(in2_slice, in2_chan, pwm_wrap + 1);

    pwm_set_enabled(in1_slice, true);
}

void drv8833_set_duty(float duty) {
    if (duty >  100.0f) duty =  100.0f;
    if (duty < -100.0f) duty = -100.0f;
    uint32_t full = pwm_wrap + 1;
    uint32_t in1_level, in2_level;
    if (duty > 0.0f) {
        in1_level = full;
        in2_level = (uint32_t)((100.0f - duty) / 100.0f * (float)full);
    } else if (duty < 0.0f) {
        in1_level = (uint32_t)((100.0f + duty) / 100.0f * (float)full);
        in2_level = full;
    } else {
        in1_level = full; in2_level = full;   // brake
    }
    pwm_set_chan_level(in1_slice, in1_chan, in1_level);
    pwm_set_chan_level(in2_slice, in2_chan, in2_level);
}