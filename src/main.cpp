
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
#include "Base2Head.hpp"
#include "Head2Base.hpp"

// Function declaration
void gnc_task_run();
void blink();
void pub_slow();
void pub_fast();

// Serial callbacks
void serial_cmd_recv();
size_t read_bytes(byte *buffer, size_t n);

// Task configuration
float gnc_task_dt = 0.01f;

Scheduler runner;

Task gnc_task((int)(gnc_task_dt * 1000.0f), TASK_FOREVER, &gnc_task_run);
Task blink_task(500, TASK_FOREVER, &blink);
Task pub_slow_task(1000, TASK_FOREVER, &pub_slow);
Task pub_fast_task(10, TASK_FOREVER, &pub_fast);

// Health LED
bool blinkState = false;

// System status is initialising
uint8_t system_status_i = BASE_STATUS_PREINIT;
uint8_t system_err_i = BASE_ERR_NONE;
uint8_t seq_num_slow = 0U;
uint8_t seq_num_fast = 0U;
uint8_t seq_num_cmd = 255U;
uint16_t num_bytes_written_prev = 0U;

// Global variables
double pitch_cur_deg = 0.0;
double pitch_tgt_deg = 0.0;
double speed = 0.0;

// Instantiate controller
double Kp = 2, Ki = 0, Kd = 0;
PID ctl_pid(&pitch_cur_deg, &speed, &pitch_tgt_deg,
            Kp, Ki, Kd, DIRECT);

void setup()
{
  // Initialise serial communication
  Serial.begin(115200);

  // Initialise tasks
  nav_init(gnc_task_dt);
  ctl_init();

  ctl_pid.SetMode(AUTOMATIC);
  ctl_pid.SetSampleTime(10);

  runner.init();

  runner.addTask(gnc_task);
  runner.addTask(blink_task);
  runner.addTask(pub_slow_task);
  runner.addTask(pub_fast_task);

  gnc_task.enable();
  blink_task.enable();
  pub_slow_task.enable();
  pub_fast_task.enable();

  // Initialising filter
  system_status_i = BASE_STATUS_FILTINIT + BASE_ERR_NONE;
}

void loop()
{
  runner.execute();
  serial_cmd_recv();
}

// From SimpleSerialProtocol Core: Serial.readBytes is SLOW
// this one is much faster, but has no timeout
size_t read_bytes(byte *buffer, size_t n)
{
  size_t num_bytes_read = 0;
  int c;
  while (num_bytes_read < n)
  {
    c = Serial.read();
    if (c < 0)
      break;
    *buffer++ = (byte)c; // buffer[i] = (int8_t)c;
    num_bytes_read++;
  }
  return num_bytes_read;
}

void gnc_task_run()
{
  // Update pitch measurement
  nav();
  pitch_cur_deg = nav_get_pitch();
  ctl_pid.Compute();

  // Update motor speed
  if (fabs(pitch_cur_deg) > 0.3f)
  {
    ctl_set_motor_speed(speed,
                        speed);
  }
  else
  {
    ctl_set_motor_speed(0.0f, 0.0f);
  }

  // Update status
  if (nav_get_filter_init())
  {
    system_status_i = BASE_STATUS_RUNNING;
  }
  else
  {
    system_status_i = BASE_STATUS_FILTINIT;
  }
}

void blink()
{
}

void pub_slow()
{
  // Compute sequence number
  if (seq_num_slow == UINT8_MAX)
  {
    seq_num_slow = 0U;
  }
  else
  {
    ++seq_num_slow;
  }

  // Build message and send
  Base2HeadSlow msg;
  msg.forebyte = BASE2HEAD_FOREBYTE_SLOW;
  msg.seq = seq_num_slow;
  msg.status = (system_status_i & 0x0F) + ((system_err_i & 0x0F) << 4);
  msg.batt_soc = bat_get_state_of_charge();
  msg.batt_volt = bat_get_last_voltage();
  msg.pitch_cmd = pitch_tgt_deg;

  Serial.write((uint8_t *)&msg, sizeof(msg));
  // Blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
}

void pub_fast()
{
  // Compute sequence number
  if (seq_num_fast == UINT8_MAX)
  {
    seq_num_fast = 0U;
  }
  else
  {
    ++seq_num_fast;
  }

  // Build message and send
  Base2HeadFast msg;
  msg.forebyte = BASE2HEAD_FOREBYTE_FAST;
  msg.seq = seq_num_fast;
  msg.status = (system_status_i & 0x0F) + ((system_err_i & 0x0F) << 4);
  msg.acc_count = (uint32_t)accel_isr_count;
  msg.cam_pan_pct = 0.0f;
  msg.cam_tilt_pct = (float)sizeof(msg);
  msg.pitch_ref = (float)num_bytes_written_prev;
  msg.pitch_est = pitch_cur_deg;
  msg.speed_cmd[0] = speed;
  msg.speed_cmd[1] = speed;
  msg.acc_x_mes = ax;
  msg.acc_y_mes = ay;
  msg.acc_z_mes = az;
  msg.w_mes = gy;
  num_bytes_written_prev = Serial.write((uint8_t *)&msg, sizeof(msg));
}

void serial_error_cb(uint8_t err_num_i)
{
  system_err_i = BASE_ERR_SERIAL;
}

void serial_cmd_recv()
{
  Head2BaseCommand *msg = NULL;
  byte recv_buf[SERIAL_RX_BUFFER_SIZE];
  size_t num_bytes_avail = Serial.available();
  size_t buf_iter = 0;
  if (num_bytes_avail >= sizeof(Head2BaseCommand))
  {
    // Read the entire serial buffer and look for
    // the command packet start byte
    read_bytes(recv_buf, num_bytes_avail);
    while (buf_iter < SERIAL_RX_BUFFER_SIZE)
    {
      if (recv_buf[buf_iter] == HEAD2BASE_FOREBYTE_CMD)
      {
        msg = (Head2BaseCommand *)recv_buf;
      }
      ++buf_iter;
    }
    // Interpret message if start of packet was found
    if (msg)
    {
      // Check which command was received
      switch (msg->type)
      {
      case InitialiseFilter:
        nav_set_filter_gain(msg->val1);
        nav_set_avg_len(msg->val2);
        nav_reset_filter();
        break;
      case PanTiltAbsCamera:
        break;
      case PanTiltRelCamera:
        break;
      case LevelCamera:
        break;
      case SetSpeed:
        break;
      case SetTurnRate:
        break;
      default:
        break;
      }
    }
  }
}
