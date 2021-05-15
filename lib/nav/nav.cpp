#include "Arduino.h"
#include "ADXL345.h"
#include "I2Cdev.h"
#include "Wire.h"

#include "nav.h"

#define GYRO_PIN A7

// Complementary filter
float _acc_coeff = 0.005f;
float _gyr_coeff = 1.0f - _acc_coeff;
float _delta_t = 0.01f;
float _freq = 1.0f / _delta_t;

// Accelerometer board from FIRST Robotics 2012
// http://www.team358.org/files/programming/ControlSystem2015-2019/specs/Accelerometer-Gyro.pdf
// I2C address hard-wired to ALT high = 0x1D
ADXL345 accel(ADXL345_ADDRESS_ALT_HIGH);
int16_t ax, ay, az;

// Gyroscope
int16_t gy = 0;
int16_t _gy_bias = 0;
float _gy_sum;
const float gyr_analog2degps = 5.0f / (1024.0f * 0.007f);

// Initialisation
float accel_isr_count = 0.0f;
bool _filter_init_b = false;
int16_t _avg_len = 200;
float _pitch_avg;

// Output
float _pitch_deg_f32 = 0.0f;

float nav_get_pitch()
{
  return _pitch_deg_f32;
}

float nav_get_filter_gain()
{
  return _acc_coeff;
}

bool nav_get_filter_init()
{
  return _filter_init_b;
}

void nav_reset_filter()
{
  _filter_init_b = false;
  accel_isr_count = 0;
  _gy_bias = 0;
  _pitch_avg = 0.0f;
  return;
}

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy)
{
  long squaresum = (long)ay * ay + (long)az * az;
  float pitch_gyr = (((float)gy * gyr_analog2degps) * _delta_t);
  float pitch_acc = atan((float)ax / sqrt((float)squaresum)) * RAD_TO_DEG;

  // Update pitch with measurements when valid
  if(!_filter_init_b) {
    // When initialising, the body is assumed to be static 
    pitch = pitch_acc;
  }
  if(abs(pitch_gyr) <= 180.0f) {
    pitch += pitch_gyr;
  } 
  if(abs(pitch_acc) <= 180.0f) {
    pitch = _gyr_coeff * pitch + _acc_coeff * pitch_acc;
  }
}

void nav_set_filter_gain(float acc_coeff_upd)
{
  _acc_coeff = acc_coeff_upd;
  _gyr_coeff = 1.0f - _acc_coeff;
}

void nav_set_avg_len(float avg_len_upd)
{
  float avg_len_count = avg_len_upd * _freq;
  if (avg_len_count < 0.0f)
  {
    _avg_len = 0U;
  }
  else if (avg_len_count > (float)UINT16_MAX)
  {
    _avg_len = UINT16_MAX;
  }
  else
  {
    _avg_len = (uint16_t)avg_len_count;
  }
}

bool nav_init(float dt)
{

  // Set function sample time
  _delta_t = dt;
  _freq = 1.0f / _delta_t;

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

  nav_reset_filter();

  return accel.testConnection();
}

void nav()
{

  if (digitalRead(2))
  {
    // Read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    accel_isr_count += 1.0f;

    if (_filter_init_b)
    {
      /* Axes are as follows:
      /  X points down
      /  Y points backward
      /  Z points left */
      gy = analogRead(GYRO_PIN) - _gy_bias;
      
      complementary_filter_step(_pitch_deg_f32, -ay, -az, ax, gy);
    }
    else
    {
      // Find offset / bias
      gy = analogRead(GYRO_PIN);
      _gy_sum += (float)gy;
      
      // When initialising, complementary filter only uses accelerometer
      complementary_filter_step(_pitch_deg_f32, -ay, -az, ax, gy);
      _pitch_avg += _pitch_deg_f32;

      if (accel_isr_count >= _avg_len)
      {
        _gy_bias = (int16_t)(_gy_sum / accel_isr_count);
        _pitch_avg = _pitch_avg / accel_isr_count;

        _filter_init_b = true;
      }
      _pitch_deg_f32 = 0.0f;
    }
  }
  return;
}