#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <EEPROM.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>

#include "PwmIn.h"

#define NUM_MOTORS 4
#define PWM_FREQ 2000
#define EEPROM_ADDR 0
#define EEPROM_SIZE 64

#define ESTOP_PIN 14


// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() {
  while (1) {
    delay(100);
  }
}

/* ----------------------------- Pin Definitions ---------------------------- */

const uint8_t pwmA[NUM_MOTORS] = {2, 4, 6, 8};   // H-Bridge PWM A
const uint8_t pwmB[NUM_MOTORS] = {3, 5, 7, 9};   // H-Bridge PWM B
uint encPins[NUM_MOTORS] = {10, 11, 12, 13};     // AS5048A PWM
PwmIn encoders(encPins, NUM_MOTORS);

/* ----------------------------- Motor Variables ---------------------------- */
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

bool estop_active = false;


/* ------------------------------- ROS Objects ------------------------------ */

rcl_publisher_t log_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t target_sub;
std_msgs__msg__Float32MultiArray target_msg;

rcl_subscription_t pid_sub;
std_msgs__msg__Float32MultiArray pid_msg;

rcl_subscription_t offset_sub;
std_msgs__msg__Float32MultiArray offset_msg;

rcl_publisher_t angle_pub;
std_msgs__msg__Float32MultiArray angle_msg;



/* ---------------------------- EEPROM functions ---------------------------- */

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

/* ------------------------------ Control Logic ----------------------------- */

void setMotorPWM(uint8_t id, float effort) {
  if (estop_active) {
    // Active braking to stop as fast as possible when estop triggered
    analogWrite(pwmA[id], 255);
    analogWrite(pwmB[id], 255);
    return;
  }

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

    if (estop_active) { // set target to current position if estop
      target_angles[i]=current_angles[i];
    }

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

/* ------------------------------ ROS callbacks ----------------------------- */

void target_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  int n = min((int)msg->data.size, NUM_MOTORS);
  for (int i = 0; i < n; ++i) {
    target_angles[i] = msg->data.data[i];
  }
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
    for (int i = 0; i < NUM_MOTORS; i++) {
      angle_msg.data.data[i] = current_angles[i];
    }
    rcl_publish(&angle_pub, &angle_msg, NULL);
    last_control_time = now;
  }
}

void estop_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  ros_estop_active = msg->data;
}


/* ---------------------------------- Setup --------------------------------- */

void setup() {
  // Pin config
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(pwmA[i], OUTPUT);
    pinMode(pwmB[i], OUTPUT);
    pinMode(encPins[i], INPUT);
  }
  pinMode(ESTOP_PIN, INPUT_PULLUP);  // Active-low NC switch
  analogWriteFreq(PWM_FREQ);

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();

  // Serial, innit
  Serial.begin(115200);

  /* ----------------------------- micro-ROS setup ---------------------------- */

  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // Setup offset control
  RCCHECK(rclc_subscription_init_default(&offset_sub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/steering/zero_offsets"));
  RCCHECK(rclc_executor_add_subscription(&executor, &offset_sub, &offset_msg, &offset_callback, ON_NEW_DATA));

  // Setup publishing of current angles
  angle_msg.data.capacity = NUM_MOTORS;
  angle_msg.data.size = NUM_MOTORS;
  angle_msg.data.data = (float*) malloc(NUM_MOTORS * sizeof(float));

  RCCHECK(rclc_publisher_init_default(&angle_pub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/steering/current_angles"));

  // Setup steering control
  RCCHECK(rclc_subscription_init_default(&target_sub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/steering/target_angle"));
  RCCHECK(rclc_executor_add_subscription(&executor, &target_sub, &target_msg, &target_callback, ON_NEW_DATA));


  // Set up PID control
  RCCHECK(rclc_subscription_init_default(&pid_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/steering/pid_params"));

  // Setup PID timer
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

  RCCHECK(rclc_subscription_init_default(&estop_sub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  "/steering/estop"));
  RCCHECK(rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &estop_callback, ON_NEW_DATA));

}

// ---------- Loop ----------
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  estop_active = digitalRead(ESTOP_PIN) == LOW || ros_estop_active;
}
