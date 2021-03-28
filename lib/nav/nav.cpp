#include "Arduino.h"
#include "ADXL345.h"
#include "I2Cdev.h"
#include "Wire.h"

#include "nav.h"

#define GYRO_PIN A7 

// Complementary filter
float pitch_acc;
float acc_coeff = 0.02f;
float gyr_coeff = 0.98f;
float delta_t = 0.01f;

// Accelerometer board from FIRST Robotics 2012
// http://www.team358.org/files/programming/ControlSystem2015-2019/specs/Accelerometer-Gyro.pdf
// I2C address hard-wired to ALT high = 0x1D
ADXL345 accel(ADXL345_ADDRESS_ALT_HIGH);

float accel_isr_count = 0.0f;
bool process_data_b = false;
bool filter_init_b = false;

int16_t ax, ay, az;
int16_t ay_offset, az_offset;
float ay_sum, az_sum;

// Gyroscope 
int16_t gy = 0;  
int16_t gy_bias = 0;  
float gy_sum;  

int16_t avg_len = 300;

float pitch = 0.0f;

// Accelerometer data ready interrupt
void accel_isr() {
  process_data_b = true;
  return;
}

float nav_get_pitch() {
  return pitch;
}

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy) {
  long squaresum = (long)ay * ay + (long)az * az;
  pitch += ((-gy / 32.8f) * (delta_t / 1000000.0f));
  pitch_acc = atan(ax / sqrt(squaresum)) * RAD_TO_DEG;
  pitch = gyr_coeff * pitch + acc_coeff * pitch_acc;
}

bool nav_init() {
    // Join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // Initialize device
  Serial.println("Initializing I2C devices...");
  accel.initialize();
  accel.setRate(ADXL345_RATE_100);
  accel.setRange(ADXL345_RANGE_2G);
  accel.setFIFOMode(ADXL345_FIFO_MODE_BYPASS);

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful"
                                        : "ADXL345 connection failed");

  // Configure interrupt for accelerometer data ready
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), accel_isr, RISING);
  accel.setIntDataReadyPin(0);
  accel.getAcceleration(&ax, &ay, &az);
  accel.setIntDataReadyEnabled(true);

  // Reset the sensor measurements accumulators
  ay_sum = 0.0f;
  az_sum = 0.0f;
  gy_sum = 0.0f;  

  return accel.testConnection();
}

void nav() {

  if (process_data_b) {
    // Read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    // Remove offsets (except for down-pointing X axis)
    ay -= ay_offset;
    az -= az_offset;
    accel_isr_count += 1.0f;
    process_data_b = false;
    gy = analogRead(GYRO_PIN) - gy_bias;

    if (filter_init_b) {
      /* Axes are as follows:
      /  X points down
      /  Y points backward
      /  Z points left */
      complementary_filter_step(pitch, -ay, -az, ax, gy);
    } else {
      // Find offset / bias
      gy_sum += (float)gy;
      ay_sum += (float)ay;
      az_sum += (float)az;
      if(accel_isr_count >= avg_len) {
        ay_offset = (int16_t)(ay_sum / accel_isr_count);
        az_offset = (int16_t)(az_sum / accel_isr_count);

        Serial.println("Accelerometer offsets (y, z):");
        Serial.print(ay_sum / accel_isr_count);
        Serial.print(", ");
        Serial.println(az_sum / accel_isr_count);

        gy_bias = (int16_t)(gy_sum / accel_isr_count);
        Serial.print("Gyroscope bias set to ");
        Serial.println((gy_sum / accel_isr_count));

        filter_init_b = true;
      }
    }
  }
  
  return;
}