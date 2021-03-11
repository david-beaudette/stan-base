  
/** STAN THE STANDING ROBOT
   Program for the base / low-level controller 
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "TaskScheduler.h"

// Project headers
#include "nav.h"

void blink();
void show();

Scheduler runner;

Task nav_task(5, TASK_FOREVER, &nav);
Task blink_task(1000, TASK_FOREVER, &blink);
Task show_task(200, TASK_FOREVER, &show);

// Health LED
bool blinkState = false;


void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  nav_init();
  
  runner.init();
  Serial.println("Initialized scheduler.");
  
  runner.addTask(nav_task);
  runner.addTask(blink_task);
  runner.addTask(show_task);
  Serial.println("Added tasks.");
  
  nav_task.enable();
  blink_task.enable();
  show_task.enable();
  Serial.println("Enabled tasks.");
}

void loop() {
  runner.execute();
}

void blink() {
  // Blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
}

void show() {
  // Display tab-separated accel x/y/z values
  Serial.print("count:\t");
  Serial.print((int)accel_isr_count);
  Serial.print("\tpitch:\t");
  Serial.print(pitch);
  Serial.print("\tgyro:\t");
  Serial.print(gy);
  Serial.print("\taccel:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
}
