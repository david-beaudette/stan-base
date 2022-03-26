/** STAN THE STANDING ROBOT
   Program to test the motor control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>

#include "ctl.h"
#include "lcd.h"
#include "nav.h"
#include "pico/stdlib.h"
#include "pid.h"
#include "uip.h"
#include "uti.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  bool led_state_b = true;
  float pot_pct_f32 = 0.0f;
  PidData* ctl_pid;
  float Ku_f32 = 0.0f;
  float rpy_est_f32[3], ome_est_f32[3];
  uint8_t gyro_state_ui8;
  uint8_t accel_state_ui8;
  uint8_t mag_state_ui8;

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  uip_init();

  // Initialise sensor in accelerometer only mode
  while (!lcd_init()) {
    printf("LCD init failed, retrying in 2 s\n");
    sleep_ms(2000);
  }

  lcd_home();
  lcd_print("Kp = ");
  sleep_ms(2000);

  ctl_init();
  ctl_pid = ctl_get_pid();

  // The objective is to find Ku and Tu for the system;
  // use PID with direct input to Kp with the "KU" mode
  pid_set_gains(ctl_pid, Ku_f32, 0.0f, PID_TM_KU);

  // Setup the pitch estimator
  if (!nav_init()) {
    printf("LCD init failed, retrying in 2 s\n");
  }
  // Infinite Loop
  while (1) {
    gpio_put(LED_PIN, led_state_b);
    led_state_b = !led_state_b;

    // Convert pot value to Kp
    pot_pct_f32 = uip_get_pot_pct();
    Ku_f32 = map(pot_pct_f32, 0.0f, 100.0f, 0.01f, 10.0f);

    // Get pitch and pitch rate estimate
    nav_get_rpy(rpy_est_f32);
    nav_get_ome(ome_est_f32);
    nav_get_calib_state(&gyro_state_ui8, &accel_state_ui8, &mag_state_ui8);

    sleep_ms(500);
  }

  return 0;
}

void print_state(const float* Ku_f32,
                 const float* pitch_f32,
                 const uint8_t gyro_state_ui8,
                 const uint8_t accel_state_ui8,
                 const uint8_t mag_state_ui8) {

  static char lcd_str[20];

  // Line 0 shows Ku and pitch
  lcd_setCursor(0, 0);
  sprintf(lcd_str, "Ku %0.2f < %0.2f%c", Ku_f32, pitch_f32, (char)223);
  lcd_print(lcd_str);
  // Line 1 shows BNO055 state
  lcd_setCursor(1, 0);
  sprintf(lcd_str, "G %hhu|A %hhu|M %hhu", gyro_state_ui8, accel_state_ui8,
          mag_state_ui8);
  lcd_print(lcd_str);
  printf("%s\n", lcd_str);
}