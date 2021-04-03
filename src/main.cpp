  
/** STAN THE STANDING ROBOT
   Program for the base / low-level controller 
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "TaskScheduler.h"

// Project headers
#include "nav.h"
#include "ctl.h"
#include "bat.h"

void blink();
void show();
void gnc_task_run();

float gnc_task_dt = 0.01f;

Scheduler runner;

Task gnc_task((int)(gnc_task_dt * 1000.0f), TASK_FOREVER, &gnc_task_run);
Task blink_task(500, TASK_FOREVER, &blink);
Task show_task(1000, TASK_FOREVER, &show);

// Health LED
bool blinkState = false;

float pitch_deg_f32 = 0.0f;

void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  nav_init(gnc_task_dt);
  ctl_init();
  
  runner.init();
  Serial.println("Initialized scheduler.");
  
  runner.addTask(gnc_task);
  runner.addTask(blink_task);
  runner.addTask(show_task);
  Serial.println("Added tasks.");
  
  gnc_task.enable();
  blink_task.enable();
  show_task.enable();
  Serial.println("Enabled tasks.");

  Serial.print("GNC task will run every ");
  Serial.print(gnc_task.getInterval());
  Serial.println(" ms.");

  check_bat_level_verbose();
}

void loop() {
  runner.execute();
}

void gnc_task_run() {
  // Update pitch measurement
  nav();
  pitch_deg_f32 = nav_get_pitch();

  // Update motor speed
  if(fabs(pitch_deg_f32) > 2.0f) {
    ctl_set_motor_speed(-pitch_deg_f32 * 0.5f, 
                        -pitch_deg_f32 * 0.5f);
  }
  else {
    ctl_set_motor_speed(0.0f, 0.0f);
  }
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
  Serial.print(pitch_deg_f32);
  Serial.print("\tgyro:\t");
  Serial.print(gy);
  Serial.print("\taccel:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
}
