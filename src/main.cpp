
//
#include "Arduino.h"

#include "hal.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel(ADXL345_ADDRESS_ALT_HIGH);

int16_t ax, ay, az;

bool blinkState = false;
uint32_t blink_count_i32 = 0U;
int gyro_val = 0; // variable to store the value coming from the sensor

uint32_t accel_isr_count_i32 = 0U;
bool process_data_b = false;

float pitch = 0;
float pitchAcc;
float P_CompCoeff = 0.98;

void accel_isr()
{
  ++accel_isr_count_i32;
  process_data_b = true;
  return;
}

void ComplementaryFilterStep(int ax, int ay, int az, int gy, int gz)
{
  long squaresum = (long)ay * ay + (long)az * az;
  float delta_t = 0.01;
  pitch += ((-gy / 32.8f) * (delta_t / 1000000.0f));
  pitchAcc = atan(ax / sqrt(squaresum)) * RAD_TO_DEG;
  pitch = P_CompCoeff * pitch + (1.0f - P_CompCoeff) * pitchAcc;
}

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accel.initialize();
  accel.setRate(ADXL345_RATE_100);
  accel.setRange(ADXL345_RANGE_2G);
  accel.setFIFOMode(ADXL345_FIFO_MODE_BYPASS);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure interrupt for accelerometer data ready
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), accel_isr, RISING);
  accel.setIntDataReadyPin(0);
  accel.getAcceleration(&ax, &ay, &az);
  accel.setIntDataReadyEnabled(true);
}

void loop()
{
  ++blink_count_i32;

  if (blink_count_i32 >= 200)
  {
    // display tab-separated accel x/y/z values
    Serial.print("count:\t");
    Serial.print(accel_isr_count_i32);
    Serial.print("\tpitch:\t");
    Serial.print(gyro_val);
    Serial.print("\tgyro:\t");
    Serial.print(gyro_val);
    Serial.print("\taccel:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.println(az);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
    blink_count_i32 = 0U;
  }

  if (process_data_b)
  {
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    process_data_b = false;
    gyro_val = analogRead(GYRO_PIN);
  }
  delay(5);
}