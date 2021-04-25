#include "Arduino.h"
#include "ADXL345.h"
#include "I2Cdev.h"
#include "Wire.h"

#include "nav.h"

#define GYRO_PIN A7

// Complementary filter
float acc_coeff = 0.025f;
float gyr_coeff = 1.0f - acc_coeff;
float delta_t = 0.01f;
float freq = 1.0f / delta_t;
const float gyr_analog2degps = 5.0f / (1024.0f * 0.007f);

// Accelerometer board from FIRST Robotics 2012
// http://www.team358.org/files/programming/ControlSystem2015-2019/specs/Accelerometer-Gyro.pdf
// I2C address hard-wired to ALT high = 0x1D
ADXL345 accel(ADXL345_ADDRESS_ALT_HIGH);

float accel_isr_count = 0.0f;
bool filter_init_b = false;

int16_t ax, ay, az;
int16_t ay_offset, az_offset;
float ay_sum, az_sum;

// Gyroscope
int16_t gy = 0;
int16_t gy_bias = 0;
float gy_sum;

int16_t avg_len = 100;

float _pitch_deg_f32 = 0.0f;

float nav_get_pitch()
{
  return _pitch_deg_f32;
}

bool nav_get_filter_init()
{
  return filter_init_b;
}

void nav_reset_filter()
{
  filter_init_b = false;
  accel_isr_count = 0;
  return;
}

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy)
{
  long squaresum = (long)ay * ay + (long)az * az;
  float pitch_gyr = (((float)gy * gyr_analog2degps) * delta_t);
  float pitch_acc = atan((float)ax / sqrt((float)squaresum)) * RAD_TO_DEG;

  // Update pitch with measurements when valid
  if(abs(pitch_gyr) <= 180.0f) {
    pitch += pitch_gyr;
  } 
  if(abs(pitch_acc) <= 180.0f) {
    pitch = gyr_coeff * pitch + acc_coeff * pitch_acc;
  }
}

void nav_set_filter_gain(float acc_coeff_upd)
{
  acc_coeff = acc_coeff_upd;
  gyr_coeff = 1.0f - acc_coeff;
}

void nav_set_avg_len(float avg_len_upd)
{
  float avg_len_count = avg_len_upd * freq;
  if (avg_len_count < 0.0f)
  {
    avg_len = 0U;
  }
  else if (avg_len_count > (float)UINT16_MAX)
  {
    avg_len = UINT16_MAX;
  }
  else
  {
    avg_len = (uint16_t)avg_len_count;
  }
}

bool nav_init(float dt)
{

  // Set function sample time
  delta_t = dt;
  freq = 1.0f / delta_t;

  // Join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // Initialize device
  accel.initialize();
  accel.setRate(ADXL345_RATE_100);
  accel.setRange(ADXL345_RANGE_2G);
  accel.setFIFOMode(ADXL345_FIFO_MODE_BYPASS);

  // Configure interrupt for accelerometer data ready
  pinMode(2, INPUT_PULLUP);
  accel.setIntDataReadyPin(0);
  accel.getAcceleration(&ax, &ay, &az);
  accel.setIntDataReadyEnabled(true);

  // Reset the sensor measurements accumulators
  ay_sum = 0.0f;
  az_sum = 0.0f;
  gy_sum = 0.0f;

  return accel.testConnection();
}

void nav()
{

  if (digitalRead(2))
  {
    // Read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    // Remove offsets (except for down-pointing X axis)
    ay -= ay_offset;
    az -= az_offset;
    accel_isr_count += 1.0f;
    gy = analogRead(GYRO_PIN) - gy_bias;

    if (filter_init_b)
    {
      /* Axes are as follows:
      /  X points down
      /  Y points backward
      /  Z points left */
      complementary_filter_step(_pitch_deg_f32, -ay, -az, ax, gy);
    }
    else
    {
      // Find offset / bias
      gy_sum += (float)gy;
      ay_sum += (float)ay;
      az_sum += (float)az;
      if (accel_isr_count >= avg_len)
      {
        ay_offset = (int16_t)(ay_sum / accel_isr_count);
        az_offset = (int16_t)(az_sum / accel_isr_count);

        gy_bias = (int16_t)(gy_sum / accel_isr_count);

        filter_init_b = true;
      }
    }
  }
  return;
}