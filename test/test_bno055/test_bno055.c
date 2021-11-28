/** STAN THE STANDING ROBOT
   Program to test the Bosch BNO055 9DoF sensor interface
   by David Beaudette
**/

// Libraries
#include <stdio.h>

#include "bno055.h"
#include "pico/stdlib.h"
#include "uip.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  uip_init();

  // Initialise sensor in accelerometer only mode
  while (!bno055_init()) {
    printf("BNO055 init failed, retrying in 2 s\n");
    sleep_ms(2000);
  }  

  float acc_f32[3], yrp_f32[3], ome_f32[3];
  uint8_t system, gyro, accel, mag;

  // Chip IDs
  printf("BNO055 chip ID: %d, gyro %d, accel %d, mag %d\n",
          bno055_read8(BNO055_CHIP_ID_ADDR), 
          bno055_read8(BNO055_GYRO_REV_ID_ADDR),
          bno055_read8(BNO055_ACCEL_REV_ID_ADDR),
          bno055_read8(BNO055_MAG_REV_ID_ADDR));

  // Firmware version
  printf("Firmware version %d.%d\n", 
          bno055_read8(BNO055_SW_REV_ID_MSB_ADDR),
          bno055_read8(BNO055_SW_REV_ID_LSB_ADDR));

  // Infinite Loop
  while (1) {
    // System state
    printf("Status: %d, error %d, temp %d degC\n", 
           bno055_read8(BNO055_SYS_STAT_ADDR),
           bno055_read8(BNO055_SYS_ERR_ADDR),
           bno055_read8(BNO055_TEMP_ADDR));

    // Calibration state
    // It was found that system bits are not crucial and may
    // toggle during operation. As long as the 3 sensor 
    // calibration bits are == 3, RPY should be accurate
    bno055_get_calib_state(&system, &gyro, &accel, &mag);
    printf("Calib: system %d, gyro %d, accel %d, mag %d\n", 
          system, gyro, accel, mag);

    // Estimated states
    bno055_get_linear_accel(acc_f32);
    printf("accX: %6.2f    accY: %6.2f     accZ: %6.2f m/s2\n", 
           acc_f32[0], acc_f32[1], acc_f32[2]);
    bno055_get_grv_vec(acc_f32);
    printf("grvX: %6.2f    grvY: %6.2f     grvZ: %6.2f m/s2\n", 
           acc_f32[0], acc_f32[1], acc_f32[2]);
    bno055_get_rpy(yrp_f32);
    printf("roll: %6.2f    pitch: %6.2f    yaw: %6.2f deg\n", 
           -yrp_f32[1], yrp_f32[2], yrp_f32[0]);
    bno055_get_ome(ome_f32);
    printf("omeX: %6.2f    omeY: %6.2f     omeZ: %6.2f deg\n\n", 
           ome_f32[0], ome_f32[1], ome_f32[2]);
    sleep_ms(500);

    // TODO: Save calibration to flash and retrieve at the end
  }

  return 0;
}