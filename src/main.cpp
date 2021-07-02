
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
#include "fletcher_impl.hpp"

// Function declaration
void gnc_task_run();
void blink();
void pub_slow();
void pub_fast();
void check_sys();

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
Task check_sys_task(1000, TASK_FOREVER, &check_sys);

// Health LED
bool blink_state = false;

// System status is initialising
uint8_t system_status_i = BASE_STATUS_PREINIT;
uint8_t system_err_i = BASE_ERR_NONE;
uint8_t seq_num_slow = 0U;
uint8_t seq_num_fast = 0U;
uint8_t seq_recv_last = 255U;
uint16_t crc_error_count = 0U;
uint16_t num_bytes_written_prev = 0U;

// Global variables
double pitch_cur_deg = 0.0;
double pitch_tgt_deg = 0.0;
double speed = 0.0;
bool motors_enabled = false;

// Instantiate controller
double Kp = 0.1, Ki = 0, Kd = 0;
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
  ctl_pid.SetOutputLimits(-1000.0, 1000.0);

  runner.init();

  runner.addTask(gnc_task);
  runner.addTask(blink_task);
  runner.addTask(pub_slow_task);
  runner.addTask(pub_fast_task);
  runner.addTask(check_sys_task);

  gnc_task.enable();
  blink_task.enable();
  pub_slow_task.enable();
  pub_fast_task.enable();
  check_sys_task.enable();
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

  // Update controller if motors are on and
  // initialisation phase is complete
  if (nav_get_filter_init())
  {
    pitch_cur_deg = nav_get_pitch();

    if (motors_enabled)
    {
      system_status_i = BASE_STATUS_RUNNING;
      ctl_pid.Compute();
      ctl_set_motor_speed(speed,
                          speed);
    }
    else
    {
      system_status_i = BASE_STATUS_MOTOR_OFF;
    }
  }
  else
  {
    system_status_i = BASE_STATUS_FILTINIT;
  }
}

void blink()
{
  // Blink LED to indicate activity
  blink_state = !blink_state;
  digitalWrite(LED_BUILTIN, blink_state);
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
  msg.seq_recv_last = seq_recv_last;
  msg.crc_error_count = crc_error_count;
  msg.status = (system_status_i & 0x0F) + ((system_err_i & 0x0F) << 4);
  msg.batt_soc = bat_get_state_of_charge();
  msg.batt_volt = bat_get_last_voltage();
  msg.pitch_zero = pitch_tgt_deg;
  msg.pitch_filter_gain = nav_get_filter_gain();
  msg.pitch_ctl_gain_P = Kp;
  msg.pitch_ctl_gain_I = Ki;
  msg.pitch_ctl_gain_D = Kd;
  msg.crc = compute_fletcher16((uint8_t *)&msg,
                               sizeof(Base2HeadSlow) -
                                   sizeof(Head2BaseCommand::crc));

  Serial.write((uint8_t *)&msg, sizeof(msg));
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
  msg.acc_count = accel_isr_count_ui32;
  msg.cam_pan_pct = 0.0f;
  msg.cam_tilt_pct = 0.0f;
  msg.pitch_cmd = pitch_tgt_deg;
  msg.pitch_est = pitch_cur_deg;
  msg.speed_cmd[0] = speed;
  msg.speed_cmd[1] = speed;
  msg.acc_x_mes = ax;
  msg.acc_y_mes = ay;
  msg.acc_z_mes = az;
  msg.w_mes = gy * gyr_analog2degps;
  msg.crc = compute_fletcher16((uint8_t *)&msg,
                               sizeof(Base2HeadFast) -
                                   sizeof(Head2BaseCommand::crc));

  num_bytes_written_prev = Serial.write((uint8_t *)&msg, sizeof(msg));
}

void check_sys()
{
  if (bat_check_level())
  {
    system_err_i &= ~BASE_ERR_LOW_BAT;
  }
  else
  {
    system_err_i |= BASE_ERR_LOW_BAT;
  }
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
      seq_recv_last = msg->seq;
      uint16_t crc = compute_fletcher16((uint8_t *)msg,
                                        sizeof(Head2BaseCommand) -
                                            sizeof(Head2BaseCommand::crc));

      if (crc == msg->crc)
      {
        // Check which command was received
        switch (msg->type)
        {
        case InitialiseFilter:
          nav_set_filter_gain(msg->val1);
          nav_set_avg_len(msg->val2);
          nav_reset_filter();
          break;
        case TunePitchControl1:
          Kp += msg->val1;
          Ki += msg->val2;
          ctl_pid.SetTunings(Kp, Ki, Kd);
          break;
        case TunePitchControl2:
          Kd += msg->val1;
          ctl_pid.SetTunings(Kp, Ki, Kd);
          break;
        case PanTiltAbsCamera:
          break;
        case PanTiltRelCamera:
          break;
        case LevelCamera:
          break;
        case TuneZeroPitch:
          pitch_tgt_deg += msg->val1;
          break;
        case SetSpeed:
          break;
        case SetTurnRate:
          break;
        case LedBlinkRate:
          blink_task.setInterval((unsigned long)(msg->val1 * 1000.0f));
          break;
        case SetRunningState:
          motors_enabled = (msg->val1 > 0.5f);
          if (motors_enabled)
          {
            ctl_enable_motors();
          }
          else
          {
            ctl_disable_motors();
          }
          break;
        default:
          break;
        }
      }
      else
      {
        ++crc_error_count;
      }
    }
  }
}
