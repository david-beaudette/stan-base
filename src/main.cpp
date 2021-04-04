  
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
#include "PID_v1.h"

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

double pitch_cur_deg = 0.0;
double pitch_tgt_deg = 0.0;
double speed = 0.0;

// Instantiate controller
double Kp=2, Ki=0, Kd=0;
PID ctl_pid(&pitch_cur_deg, &speed, &pitch_tgt_deg, 
            Kp, Ki, Kd, DIRECT);

void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  nav_init(gnc_task_dt);
  ctl_init();
  ctl_pid.SetMode(AUTOMATIC);
  ctl_pid.SetSampleTime(10);
  
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

  bat_show_state_of_charge();
  bat_show_voltage();
}

void loop() {
  runner.execute();
}

void gnc_task_run() {
  // Update pitch measurement
  nav();
  pitch_cur_deg = nav_get_pitch();
  ctl_pid.Compute();

  // Update motor speed
  if(fabs(pitch_cur_deg) > 0.3f) {
    ctl_set_motor_speed(speed, 
                        speed);
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
  Serial.print(accel_isr_count, 0);
  Serial.print("\tpitch:\t");
  Serial.print(pitch_cur_deg);
  Serial.print("\tgyro:\t");
  Serial.print(gy);
  Serial.print("\taccel:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
}
