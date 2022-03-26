/** STAN THE STANDING ROBOT
   Program to test the motor control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"

#include "ctl.h"
#include "uip.h"
#include "lcd.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  bool led_state_b = true;
  float pot_pct_f32 = 0.0f;
  char pot_str[20];
  PidData* ctl_pid;

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
  lcd_print("Ctl Test: Pot = Kp");
  sleep_ms(2000);

  ctl_init();
  ctl_pid = ctl_get_pid();
  // The objective is to find Ku
  pid_set_gains(ctl_pid, 0.0, 0.0, PID_TM_KU);

  // Infinite Loop
  while (1) {
    gpio_put(LED_PIN, led_state_b);
    led_state_b = !led_state_b;

    pot_pct_f32 = uip_get_pot_angle();

    lcd_setCursor(1, 0);
    sprintf(pot_str, "Pot pos: %6.2f%%", pot_pct_f32);
    lcd_print(pot_str);
    printf("%s\n", pot_str);
    sleep_ms(500);
  }

  return 0;
}
