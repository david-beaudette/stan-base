/** STAN THE STANDING ROBOT
   Program to interface the Bosch BNO055 9DoF sensor
   by David Beaudette
**/

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno055.h"

static int BNO055_I2C_ADDR = 0x28;

void bno055_init(void) {

  // Setup I2C port
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4);
  gpio_pull_up(5);

  // Set orientation for Stan
  uint8_t data[2];
  data[0] = BNO055_AXIS_MAP_CONFIG_ADDR;
  data[1] = 0x24;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

  // Default Axis Signs
  data[0] = BNO055_AXIS_MAP_SIGN_ADDR;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

  return;
}

// Initialise Accelerometer Function
void accel_init(void) {
  // Check to see if connection is correct
  sleep_ms(1000);  // Add a short delay to help BNO005 boot up

  while (bno055_read8(BNO055_CHIP_ID_ADDR) != 0xA0) {
    printf("Chip ID Not Correct - Check Connection!");
    sleep_ms(5000);
  }

  // Use internal oscillator
  uint8_t data[2];
  data[0] = 0x3F;
  data[1] = 0x40;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

  // Reset all interrupt status bits
  data[0] = 0x3F;
  data[1] = 0x01;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

  // Configure Power Mode
  data[0] = 0x3E;
  data[1] = 0x00;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
  sleep_ms(50);

  // Set units to m/s^2
  data[0] = 0x3B;
  data[1] = 0b0001000;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
  sleep_ms(30);

  // Set operation to acceleration only
  data[0] = 0x3D;
  data[1] = 0x0C;
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
  sleep_ms(100);

  return;
}

void bno055_get_accel(float *acc) {
  // Store data from the 6 acceleration registers
  uint8_t accel[6];  
  // Combined 3 axis data
  int16_t accelX, accelY, accelZ;       
  // Start register address
  uint8_t val = 0x08;                  

  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &val, 1, true);
  i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, accel, 6, false);

  accelX = ((accel[1] << 8) | accel[0]);
  accelY = ((accel[3] << 8) | accel[2]);
  accelZ = ((accel[5] << 8) | accel[4]);

  acc[0] = accelX / 100.00;
  acc[1] = accelY / 100.00;
  acc[2] = accelZ / 100.00;

  return;
}

uint8_t bno055_read8(bno055_registers reg) {
  uint8_t i2c_buf_ui8[1] = {reg};
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, i2c_buf_ui8, 1, true);
  i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, i2c_buf_ui8, 1, false);
  return i2c_buf_ui8[0];
}

void bno055_write8(bno055_registers reg, uint8_t val_ui8) {
  uint8_t i2c_buf_ui8[2] = {reg, val_ui8};
  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, i2c_buf_ui8, 2, true);
  return;
}