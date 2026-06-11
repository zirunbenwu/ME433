#ifndef CURRENT_CTRL_H
#define CURRENT_CTRL_H
#include "pico/stdlib.h"
#include "hardware/timer.h"

enum mode_t { IDLE, PWM_MODE, ITEST, BUZZ };

// ---- shared control state (defined in current_ctrl.c) ----
extern volatile enum mode_t state;
extern volatile float Kp_current;
extern volatile float Ki_current;
extern volatile float desired_current;   // mA
extern volatile float duty_cycle;         // for PWM_MODE
extern volatile float BUZZ_AMP_MA;
extern volatile float BUZZ_FREQ_HZ;
extern volatile uint32_t buzz_until_ms;   // BUZZ auto-stops after this time
extern volatile float g_current_ma;       // latest current, updated in main loop

// ITEST result buffers
extern volatile float itest_desired[];
extern volatile float itest_actual[];

// ---- functions ----
void current_ctrl_start(struct repeating_timer *timer);  // install 1kHz ISR
void current_ctrl_reset(void);                            // zero integrator/phase/index

#endif