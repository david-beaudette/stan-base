
// 

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
int gyro_val = 0;  // variable to store the value coming from the sensor

float pitch=0;
float pitchAcc;
float P_CompCoeff= 0.98;
void ComplementaryFilter(int ax,int ay,int az,int gy,int gz) {
 long squaresum=(long)ay*ay+(long)az*az;
 pitch+=((-gy/32.8f)*(delta_t/1000000.0f));
 pitchAcc =atan(ax/sqrt(squaresum))*RAD_TO_DEG;
 pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
}
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accel.initialize();
    accel.setRate(ADXL345_RATE_100);
accel.setRate(ADXL345_RATE_100);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

    // configure LED for output
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

    gyro_val = analogRead(GYRO_PIN);
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);

    // display tab-separated accel x/y/z values
    Serial.print("pitch:\t");
    Serial.print(gyro_val); Serial.print("\t");
    Serial.print("gyro:\t");
    Serial.print(gyro_val); Serial.print("\t");
    Serial.print("accel:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
    delay(1000);
}