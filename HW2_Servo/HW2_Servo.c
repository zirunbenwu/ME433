#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define PWMPIN 16

// servo pulse widths in PWM counts
// with div=100 and 150MHz clock, 1 count = 1/1.5MHz ≈ 0.667us
// 1ms = 1500 counts (0 degrees)
// 1.5ms = 2250 counts (90 degrees)
// 2ms = 3000 counts (180 degrees)
#define SERVO_MIN 750
#define SERVO_MAX 3750
#define SERVO_WRAP 30000  // 1.5MHz / 50Hz = 30000 for 50Hz period

bool timer_interrupt_function(__unused struct repeating_timer *t) {
    uint16_t result1 = adc_read();
    printf("%f\r\n", (float)result1 / 4095 * 3.3);
    return true;
}

void set_servo_angle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint16_t level = SERVO_MIN + (uint16_t)((angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
    pwm_set_gpio_level(PWMPIN, level);
}

int main()
{
    stdio_init_all();

    // set up PWM for servo at 50Hz
    gpio_set_function(PWMPIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWMPIN);
    float div = 100;  // 150MHz / 100 = 1.5MHz
    pwm_set_clkdiv(slice_num, div);
    pwm_set_wrap(slice_num, SERVO_WRAP);
    pwm_set_enabled(slice_num, true);

    // start at 0 degrees
    set_servo_angle(0);

    // turn on the adc
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    // turn on a timer interrupt for ADC readings
    struct repeating_timer timer;
    add_repeating_timer_ms(-100, timer_interrupt_function, NULL, &timer);

    float angle = 0;
    float step = 1;  // degrees per step

    while (true) {
        set_servo_angle(angle);
        angle += step;

        // reverse direction at the limits
        if (angle >= 180) {
            angle = 180;
            step = -1;
        } else if (angle <= 0) {
            angle = 0;
            step = 1;
        }

        sleep_ms(20);  // 20ms delay per step, full sweep takes ~3.6 seconds
    }
}