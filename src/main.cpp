#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <EEPROM.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "PwmIn.h"

#define NUM_MOTORS 4
#define EEPROM_ADDR 0

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// ---------------- Motor Pin Definitions ----------------
const uint8_t pwmA[NUM_MOTORS] = {2, 4, 6, 8};   // H-Bridge PWM A
const uint8_t pwmB[NUM_MOTORS] = {3, 5, 7, 9};   // H-Bridge PWM B
uint encPins[NUM_MOTORS] = {10, 11, 12, 13};     // AS5048A PWM
PwmIn encoders(encPins, NUM_MOTORS);

// ---------------- Motor Constants ----------------
const float MAX_ANGLE = 170;
const float MIN_ANGLE = -170;
float offsets[NUM_MOTORS] = {0, 0, 0, 0};

struct PIDConfig {
  float kp;
  float ki;
  float kd;
};

PIDConfig pidConfig = {1.0, 0.0, 0.0};

float pid_integral[NUM_MOTORS] = {0};
float pid_last_error[NUM_MOTORS] = {0};

float target_angles[NUM_MOTORS] = {0};
float current_angles[NUM_MOTORS] = {0};

unsigned long last_control_time = 0;
const unsigned long control_interval = 20;

// ROS Variables
rcl_publisher_t log_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t subs[NUM_MOTORS];
std_msgs__msg__Float32 target_msgs[NUM_MOTORS];

rcl_subscription_t pid_sub;
std_msgs__msg__Float32MultiArray pid_msg;

// ---------- EEPROM ---------------
void saveEEPROM() {
  EEPROM.put(EEPROM_ADDR, pidConfig);
  for (int i = 0; i < NUM_MOTORS; ++i) {
    EEPROM.put(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
  }
}

void loadEEPROM() {
  EEPROM.get(EEPROM_ADDR, pidConfig);
  for (int i = 0; i < NUM_MOTORS; ++i) {
    EEPROM.get(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
  }
  if (isnan(pidConfig.kp) || pidConfig.kp < 0 || pidConfig.kp > 100) {
    pidConfig = {1.0, 0.0, 0.0};
  }
}

// ---------- Control Logic ----------
void setMotorPWM(uint8_t id, float effort) {
  effort = constrain(effort, -255, 255);
  if (effort > 0) {
    analogWrite(pwmA[id], (int)effort);
    analogWrite(pwmB[id], 0);
  } else {
    analogWrite(pwmA[id], 0);
    analogWrite(pwmB[id], (int)-effort);
  }
}

float readEncoderPWM(uint8_t pin_index) {
  float pw = encoders.read_PW(pin_index);
  float per = encoders.read_P(pin_index);
  if (per < 1e-6) return 0;
  return (pw / per) * 360.0f * (4095.0f / 4119.0f);
}

float getShortestDelta(float from, float to) {
  float delta = to - from;
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
  return delta;
}

void controlLoop() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(i);
    current_angles[i] = raw_angle - offsets[i];

    float delta = getShortestDelta(current_angles[i], target_angles[i]);
    float target_abs = current_angles[i] + delta;
    if (target_abs < MIN_ANGLE || target_abs > MAX_ANGLE) {
      delta = (delta > 0) ? delta - 360 : delta + 360;
    }

    float error = delta;
    pid_integral[i] += error * (control_interval / 1000.0f);
    float derivative = (error - pid_last_error[i]) / (control_interval / 1000.0f);
    float output = pidConfig.kp * error + pidConfig.ki * pid_integral[i] + pidConfig.kd * derivative;
    setMotorPWM(i, output);
    pid_last_error[i] = error;
  }
}

// ---------- Callbacks ----------
void angle_callback(const void *msgin, void *arg) {
  int idx = *((int *)arg);
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  target_angles[idx] = msg->data;
}

void pid_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 3) {
    pidConfig.kp = msg->data.data[0];
    pidConfig.ki = msg->data.data[1];
    pidConfig.kd = msg->data.data[2];
    saveEEPROM();
  }
}

void timer_callback(rcl_timer_t *, int64_t) {
  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    controlLoop();
    last_control_time = now;
  }
}

// ---------- Setup ----------
void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(pwmA[i], OUTPUT);
    pinMode(pwmB[i], OUTPUT);
    pinMode(encPins[i], INPUT);
  }
  analogWriteFreq(2000);

  EEPROM.begin(64);
  loadEEPROM();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "steering_controller", "", &support));

  for (int i = 0; i < NUM_MOTORS; ++i) {
    String topic = "motor" + String(i) + "/target_angle";
    RCCHECK(rclc_subscription_init_default(
      &subs[i], &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      topic.c_str()));
  }

  RCCHECK(rclc_subscription_init_default(&pid_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/steering/pid_params"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_MOTORS + 2, &allocator));

  static int idx[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; ++i) {
    idx[i] = i;
    RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subs[i], &target_msgs[i], angle_callback, &idx[i], ON_NEW_DATA));
  }

  RCCHECK(rclc_executor_add_subscription(&executor, &pid_sub, &pid_msg, &pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ---------- Loop ----------
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
