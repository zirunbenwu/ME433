#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);


// ---------- Pin definitions ----------
#define SDA_PIN       16
#define SCL_PIN       17
#define MODE_BUTTON   8    // button to GND, uses internal pull-up
#define MODE_LED      10   // LED + 330Ω to GND, ON = remote mode

// ---------- MPU6050 ----------
#define MPU_ADDR      0x68
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define WHO_AM_I      0x75

static void mpu_write(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    i2c_write_blocking(i2c_default, MPU_ADDR, buf, 2, false);
}

static uint8_t mpu_read(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, &val, 1, false);
    return val;
}

static void mpu_read_burst(uint8_t *buf) {
    uint8_t reg = ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 14, false);
}

static int16_t combine(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

static void mpu_init(void) {
    mpu_write(PWR_MGMT_1, 0x00);
    mpu_write(ACCEL_CONFIG, 0x00);
    mpu_write(GYRO_CONFIG, 0x18);
}

// Discretize a g-value into a signed mouse delta with 4 speed levels
static int8_t discretize(float g) {
    float a = fabsf(g);
    int sign = (g < 0) ? -1 : 1;
    if (a < 0.10f) return 0;
    if (a < 0.30f) return sign * 1;
    if (a < 0.60f) return sign * 3;
    return sign * 6;
}

// ---------- Mode tracking ----------
static bool remote_mode = false;

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  // ---------- I2C + MPU6050 ----------
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  mpu_init();

  // ---------- Mode button (internal pull-up) ----------
  gpio_init(MODE_BUTTON);
  gpio_set_dir(MODE_BUTTON, GPIO_IN);
  gpio_pull_up(MODE_BUTTON);

  // ---------- Mode LED ----------
  gpio_init(MODE_LED);
  gpio_set_dir(MODE_LED, GPIO_OUT);
  gpio_put(MODE_LED, 0);

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    hid_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  (void) btn;

  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_KEYBOARD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if ( btn )
      {
        uint8_t keycode[6] = { 0 };
        keycode[0] = HID_KEY_A;

        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        has_keyboard_key = true;
      }else
      {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        has_keyboard_key = false;
      }
    }
    break;

    case REPORT_ID_MOUSE:
    {
      int8_t delta_x = 0;
      int8_t delta_y = 0;

      if (remote_mode) {
        // Slow circle — advance phase each call
        static float phase = 0.0f;
        phase += 0.05f;
        if (phase > 6.2832f) phase -= 6.2832f;

        float r = 3.0f;
        delta_x = (int8_t)(r * cosf(phase));
        delta_y = (int8_t)(r * sinf(phase));
      } else {
        // IMU mode: read accel and convert to mouse delta
        uint8_t raw[14];
        mpu_read_burst(raw);
        int16_t ax_raw = combine(raw[0], raw[1]);
        int16_t ay_raw = combine(raw[2], raw[3]);
        float ax_g = ax_raw * 0.000061f;
        float ay_g = ay_raw * 0.000061f;

        delta_x = discretize(ax_g);
        delta_y = discretize(ay_g);

        // Flip X so left tilt = left cursor
        delta_x = -delta_x;
        // Uncomment if Y is also reversed:
        // delta_y = -delta_y;
      }

      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta_x, delta_y, 0, 0);
    }
    break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
      // use to avoid send multiple consecutive zero report
      static bool has_consumer_key = false;

      if ( btn )
      {
        // volume down
        uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
        tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
        has_consumer_key = true;
      }else
      {
        // send empty key report (release key) if previously has key pressed
        uint16_t empty_key = 0;
        if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
        has_consumer_key = false;
      }
    }
    break;

    case REPORT_ID_GAMEPAD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_gamepad_key = false;

      hid_gamepad_report_t report =
      {
        .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
        .hat = 0, .buttons = 0
      };

      if ( btn )
      {
        report.hat = GAMEPAD_HAT_UP;
        report.buttons = GAMEPAD_BUTTON_A;
        tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

        has_gamepad_key = true;
      }else
      {
        report.hat = GAMEPAD_HAT_CENTERED;
        report.buttons = 0;
        if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
        has_gamepad_key = false;
      }
    }
    break;

    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  // Poll the mode button with debouncing (active-low with pull-up)
  // Require the button to be stable for 3 polls (30ms) before acting
  static int debounce_counter = 0;
  static bool stable_state = true;   // true = released

  bool button_now = gpio_get(MODE_BUTTON);

  if (button_now == stable_state) {
    debounce_counter = 0;
  } else {
    debounce_counter++;
    if (debounce_counter >= 3) {
      stable_state = button_now;
      debounce_counter = 0;

      // Act only on falling edge (just pressed)
      if (!stable_state) {
        remote_mode = !remote_mode;
        gpio_put(MODE_LED, remote_mode ? 1 : 0);
      }
    }
  }

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_MOUSE, btn);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  uint8_t next_report_id = report[0] + 1u;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id, board_button_read());
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      }else
      {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}