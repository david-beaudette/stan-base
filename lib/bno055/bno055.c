/** STAN THE STANDING ROBOT
   Program to interface the Bosch BNO055 9DoF sensor
   by David Beaudette

   Adapted from Adafruit_BNO055 Arduino Library
   https://github.com/adafruit/Adafruit_BNO055
   See LICENSE file for permission details
   Copyright (c) 2018 Adafruit Industries
**/

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno055.h"

static int BNO055_I2C_ADDR = 0x28;

bool bno055_init(void) {

  // Setup I2C port
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4);
  gpio_pull_up(5);

  // Add a short delay for the BNO005 to boot up
  // POR time is typ. 400 ms + 50% margin
  sleep_ms(600);  
  if (bno055_read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    return false;
  }

  // Reset system
  bno055_set_mode(OPERATION_MODE_CONFIG);
  bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  sleep_ms(30);
  while (bno055_read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    sleep_ms(10);
  }
  sleep_ms(50);
  
  // Configure Power Mode
  bno055_write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  sleep_ms(10);

  // Set operation to CONFIG in case the RP2040 was reset
  // while the BNO055 was powered
  bno055_set_mode(OPERATION_MODE_CONFIG);

  bno055_write8(BNO055_PAGE_ID_ADDR, 0);

  // Set orientation for Stan
  uint8_t axismap_i = (0b01 << 4) | // Z axis = -Y axis
                      (0b00 << 2) | // Y axis = +Z axis
                      (0b10 << 0);  // X axis = +X axis
  bno055_write8(BNO055_AXIS_MAP_CONFIG_ADDR, axismap_i);
  bno055_write8(BNO055_AXIS_MAP_SIGN_ADDR, 0b101);

  // Set units to m/s^2
  uint8_t unitsel = (0 << 7) | // Orientation = Positive CW
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (0 << 1) | // Gyro = Degrees per second
                    (0 << 0);  // Accelerometer = m/s^2
  bno055_write8(BNO055_UNIT_SEL_ADDR, unitsel);
  sleep_ms(30);

  // Set operation to NDOF
  bno055_write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  sleep_ms(600);

  return true;
}

void bno055_get_accel(float *acc_f32) {
  uint8_t reg_i = BNO055_ACCEL_DATA_X_LSB_ADDR;                  
  bno055_read_vec(acc_f32, -0.01f, &reg_i); 
  return;
}

void bno055_get_linear_accel(float *lia_f32) {
  uint8_t reg_i = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;                  
  bno055_read_vec(lia_f32, -0.01f, &reg_i); 
  return;
}

void bno055_get_rpy(float *yrp_f32) {
  uint8_t reg_i = BNO055_EULER_H_LSB_ADDR;                  
  bno055_read_vec(yrp_f32, 1.0f/16.0f, &reg_i); 
  return;
}

void bno055_get_grv_vec(float *grv_vec_f32) {
  uint8_t reg_i = BNO055_GRAVITY_DATA_X_LSB_ADDR;                  
  bno055_read_vec(grv_vec_f32, -0.01f, &reg_i); 
  return;
}

void bno055_get_ome(float *ome_f32) {
  uint8_t reg_i = BNO055_GYRO_DATA_X_LSB_ADDR;                  
  bno055_read_vec(ome_f32, 0.01f, &reg_i); 
  return;
}

void bno055_read_vec(float *vec_f32, 
                     const float scale_fac_f32, 
                     const uint8_t *reg_i) {

  uint8_t read_buf_i[6];  
  int16_t vec_x_i16, vec_y_i16, vec_z_i16;       

  i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, reg_i, 1, true);
  i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, read_buf_i, 6, false);

  vec_x_i16 = ((int16_t)read_buf_i[0]) | (((int16_t)read_buf_i[1]) << 8);
  vec_y_i16 = ((int16_t)read_buf_i[2]) | (((int16_t)read_buf_i[3]) << 8);
  vec_z_i16 = ((int16_t)read_buf_i[4]) | (((int16_t)read_buf_i[5]) << 8);

  vec_f32[0] = (float)vec_x_i16 * scale_fac_f32;
  vec_f32[1] = (float)vec_y_i16 * scale_fac_f32;
  vec_f32[2] = (float)vec_z_i16 * scale_fac_f32;

  return;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 */
void bno055_set_mode(bno055_opmodes mode) {
  bno055_write8(BNO055_OPR_MODE_ADDR, mode);
  sleep_ms(30);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 3.10.4
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors
 *  @param  gyro
 *          Current calibration status of Gyroscope
 *  @param  accel
 *          Current calibration status of Accelerometer
 *  @param  mag
 *          Current calibration status of Magnetometer
 */
void bno055_get_calib_state(uint8_t *sys, 
                            uint8_t *gyro, 
                            uint8_t *accel, 
                            uint8_t *mag) {

  uint8_t cal_data = bno055_read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (cal_data >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (cal_data >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (cal_data >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = cal_data & 0x03;
  }
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 */
void bno055_set_calib(const bno055_offsets_t *offsets_type) {

  // Switch to config mode after saving current mode
  uint8_t mode_prev = bno055_read8(OPERATION_MODE_CONFIG);
  bno055_set_mode(OPERATION_MODE_CONFIG);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  bno055_write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type->accel_offset_x) & 0x0FF);
  bno055_write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type->accel_offset_x >> 8) & 0x0FF);
  bno055_write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type->accel_offset_y) & 0x0FF);
  bno055_write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type->accel_offset_y >> 8) & 0x0FF);
  bno055_write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type->accel_offset_z) & 0x0FF);
  bno055_write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type->accel_offset_z >> 8) & 0x0FF);

  bno055_write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type->mag_offset_x) & 0x0FF);
  bno055_write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type->mag_offset_x >> 8) & 0x0FF);
  bno055_write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type->mag_offset_y) & 0x0FF);
  bno055_write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type->mag_offset_y >> 8) & 0x0FF);
  bno055_write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type->mag_offset_z) & 0x0FF);
  bno055_write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type->mag_offset_z >> 8) & 0x0FF);

  bno055_write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type->gyro_offset_x) & 0x0FF);
  bno055_write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type->gyro_offset_x >> 8) & 0x0FF);
  bno055_write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type->gyro_offset_y) & 0x0FF);
  bno055_write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type->gyro_offset_y >> 8) & 0x0FF);
  bno055_write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type->gyro_offset_z) & 0x0FF);
  bno055_write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type->gyro_offset_z >> 8) & 0x0FF);

  bno055_write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type->accel_radius) & 0x0FF);
  bno055_write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type->accel_radius >> 8) & 0x0FF);

  bno055_write8(MAG_RADIUS_LSB_ADDR, (offsets_type->mag_radius) & 0x0FF);
  bno055_write8(MAG_RADIUS_MSB_ADDR, (offsets_type->mag_radius >> 8) & 0x0FF);

  bno055_set_mode(mode_prev);
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool bno055_get_sensor_offsets(bno055_offsets_t *offsets_type) {

  // Switch to config mode after saving current mode
  uint8_t mode_prev = bno055_read8(OPERATION_MODE_CONFIG);
  bno055_set_mode(OPERATION_MODE_CONFIG);

    /* Accel offset range depends on the G-range:
       +/-2g  = +/- 2000 mg
       +/-4g  = +/- 4000 mg
       +/-8g  = +/- 8000 mg
       +/-1§g = +/- 16000 mg */
    offsets_type->accel_offset_x = (bno055_read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                  (bno055_read8(ACCEL_OFFSET_X_LSB_ADDR));
    offsets_type->accel_offset_y = (bno055_read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                  (bno055_read8(ACCEL_OFFSET_Y_LSB_ADDR));
    offsets_type->accel_offset_z = (bno055_read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                  (bno055_read8(ACCEL_OFFSET_Z_LSB_ADDR));

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    offsets_type->mag_offset_x =
        (bno055_read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_X_LSB_ADDR));
    offsets_type->mag_offset_y =
        (bno055_read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_Y_LSB_ADDR));
    offsets_type->mag_offset_z =
        (bno055_read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_Z_LSB_ADDR));

    /* Gyro offset range depends on the DPS range:
      2000 dps = +/- 32000 LSB
      1000 dps = +/- 16000 LSB
       500 dps = +/- 8000 LSB
       250 dps = +/- 4000 LSB
       125 dps = +/- 2000 LSB
       ... where 1 DPS = 16 LSB */
    offsets_type->gyro_offset_x =
        (bno055_read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_X_LSB_ADDR));
    offsets_type->gyro_offset_y =
        (bno055_read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_Y_LSB_ADDR));
    offsets_type->gyro_offset_z =
        (bno055_read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_Z_LSB_ADDR));

    /* Accelerometer radius = +/- 1000 LSB */
    offsets_type->accel_radius =
        (bno055_read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (bno055_read8(ACCEL_RADIUS_LSB_ADDR));

    /* Magnetometer radius = +/- 960 LSB */
    offsets_type->mag_radius =
        (bno055_read8(MAG_RADIUS_MSB_ADDR) << 8) | (bno055_read8(MAG_RADIUS_LSB_ADDR));

    bno055_set_mode(mode_prev);
    return true;
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool bno055_is_fully_calibrated() {
  uint8_t system, gyro, accel, mag;

  uint8_t mode = bno055_read8(OPERATION_MODE_CONFIG);
  bno055_get_calib_state(&system, &gyro, &accel, &mag);

  switch (mode) {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: 
    /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
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

