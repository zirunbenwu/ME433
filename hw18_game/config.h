#ifndef CONFIG_H
#define CONFIG_H

// ====================================================================
// PIN MAP
//   DRV8833 : IN1=GP16, IN2=GP17  (PWM)
//   AS5600  : I2C0 SDA=GP12, SCL=GP13
//   INA219  : I2C1 SDA=GP14, SCL=GP15
//   (HX711 load cell GP18/19 unused for now)
// ====================================================================

// ---- DRV8833 H-bridge ----
#define IN1_PIN        16
#define IN2_PIN        17
#define PWM_FREQ_HZ    20000

// ---- AS5600 encoder (I2C0) ----
#define AS_I2C         i2c0
#define AS_SDA         12
#define AS_SCL         13
#define AS_FREQ        400000

// ---- INA219 current sensor (I2C1) ----
#define INA_I2C        i2c1
#define INA_SDA        14
#define INA_SCL        15
#define INA_FREQ       400000

// ---- safety / limits ----
#define DES_CUR_LIMIT_MA   400.0f
#define EINT_MAX           1000.0f

// ---- ITEST (current step test) ----
#define ITEST_SAMPLES   400
#define ITEST_SEGMENT   100
#define ITEST_CURRENT   100.0f

#endif