#include <Arduino.h>
#include <EEPROM.h>
#include "PwmIn.h"

#include <cmath>
#include <cstddef>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>


#define NUM_MOTORS 4
#define PWM_FREQ 2000
#define PWM_RANGE 1024
#define EEPROM_ADDR 0
#define EEPROM_SIZE 64

#define ESTOP_PIN 6

const float direction[NUM_MOTORS] = {-1,-1,-1,-1};
const uint8_t pwmA[NUM_MOTORS] =    {15, 3, 8, 1};
const uint8_t pwmB[NUM_MOTORS] =    {14, 4, 7, 2};
uint encPins[NUM_MOTORS] =          {29, 28, 27, 26};
PwmIn encoders(encPins, NUM_MOTORS);

const float MAX_ANGLE = 170;
const float MIN_ANGLE = -170;

float offsets[NUM_MOTORS] = {0};
float target_angles[NUM_MOTORS] = {0};
float current_angles[NUM_MOTORS] = {0};
float previous_angles[NUM_MOTORS] = {0};

struct PIDConfig {
  float kp=0.2;
  float ki=0;
  float kd=0;
};


PIDConfig pidConfig = {1.0, 0.0, 0.0};
float pid_integral[NUM_MOTORS] = {0};
float pid_last_error[NUM_MOTORS] = {0};

bool estop_active = false;
unsigned long last_control_time = 0;
const unsigned long control_interval = 20;

/* ------------------------------- ROS objects ------------------------------ */

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

/* ------------------------------- ROS Topics ------------------------------- */

rcl_publisher_t current_angles_pub;
std_msgs__msg__Float32MultiArray current_angles_msg;

rcl_subscription_t target_angles_sub;
std_msgs__msg__Float32MultiArray target_angles_msg;

rcl_subscription_t estop_sub;
std_msgs__msg__Bool estop_msg;

rcl_subscription_t pid_sub;
std_msgs__msg__Float32MultiArray pid_msg;

rcl_subscription_t offsets_sub;
std_msgs__msg__Float32MultiArray offset_msg;


void saveEEPROM() {
  EEPROM.put(EEPROM_ADDR, pidConfig);
  for (int i = 0; i < NUM_MOTORS; ++i)
    EEPROM.put(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);

  EEPROM.commit();
}


void loadEEPROM() {
  EEPROM.get(EEPROM_ADDR, pidConfig);

  // Validate PID config
  if (!isfinite(pidConfig.kp) || pidConfig.kp < 0.0f || pidConfig.kp > 100.0f ||
      !isfinite(pidConfig.ki) ||
      !isfinite(pidConfig.kd))
  {
    pidConfig = {1.0f, 0.0f, 0.0f};
  }

  // Load and validate offsets
  for (int i = 0; i < NUM_MOTORS; ++i)
  {
    float value;
    EEPROM.get(
      EEPROM_ADDR + sizeof(PIDConfig) + i * sizeof(float),
      value
    );

    // Reject uninitialized/corrupt EEPROM values
    if (!isfinite(value) || value < -360.0f || value > 360.0f)
    {
      offsets[i] = 0.0f;
    }
    else
    {
      offsets[i] = value;
    }
  }
}


void setMotorPWM(uint8_t id, float effort) {
  if (estop_active) {
    analogWrite(pwmA[id], PWM_RANGE);
    analogWrite(pwmB[id], PWM_RANGE);
    return;
  }
  int duty =constrain((int)effort, -PWM_RANGE, PWM_RANGE);
  if (effort > 0) {
    analogWrite(pwmA[id], (int)effort);
    analogWrite(pwmB[id], 0);
  } else {
    analogWrite(pwmA[id], 0);
    analogWrite(pwmB[id], (int)-effort);
  }
}


float readEncoderPWM(uint8_t pin_index) {
  // From your PwmIn library, read_PW gives the high time and read_P gives the low time.
  float high_time_s = encoders.read_PW(pin_index);
  float low_time_s = encoders.read_P(pin_index);

  // The total period is the sum of the high time and the low time.
  float total_period_s = high_time_s + low_time_s;

  // Avoid division by zero if the signal is lost.
  if (total_period_s < 1e-6) {
    return 0.0f; // Return a default/error value
  }

  // 1. Calculate the correct duty cycle (high_time / total_time)
  float duty_cycle = high_time_s / total_period_s;

  // --- DECODING LOGIC FROM AS5048 DATASHEET ---
  
  // Total clocks in one PWM frame.
  const float total_clocks = 4119.0f; // 12(Init)+4(Error)+4095(Data)+8(Exit)
  // Fixed high-time header (Init + Error_n).
  const float header_clocks = 16.0f; // 12+4
  // Number of clocks that represent the angle data range.
  const float data_clocks = 4095.0f;

  // 2. Calculate the total number of 'high' clocks in the measured pulse.
  float high_clocks_measured = duty_cycle * total_clocks;
  
  // 3. Subtract the fixed header to get the clocks that represent the angle.
  float angle_clocks = high_clocks_measured - header_clocks;

  // 4. Convert the angle clocks to degrees.
  float angle_degrees = (angle_clocks / data_clocks) * 360.0f;

  // Clamp the value to the valid 0-360 range to prevent minor measurement errors
  // from pushing it out of bounds.
  if (angle_degrees < 0.0f) angle_degrees = 0.0f;
  if (angle_degrees > 360.0f) angle_degrees = 360.0f; 

  return angle_degrees;
}


float getShortestDelta(float from, float to) {
  float delta = to - from;
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
  return delta;
}


float angleWrap(float angle)
{
  float max_ang = 180.0f;
  float min_ang = -180.0f;

  float angle_range = max_ang - min_ang;

  return std::fmod((angle - min_ang), angle_range) + min_ang;
}


void controlLoop() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(i);

    current_angles[i] = raw_angle - offsets[i];
    
    // --- NORMALIZE CURRENT ANGLE to -180 to 180 range ---
    current_angles[i] = angleWrap(current_angles[i]);
    
    // if (previous_angles[i] > 135.0f and current_angles[i] < -135.0f)
    // {
    //   if (wrapping[i] < 1) wrapping[i] += 1;
    //   saveEEPROM();
    // }
    // else if (previous_angles[i] < -135.0f and current_angles[i] > 135.0f)
    // {
    //   if (wrapping[i] > -1) wrapping[i] -= 1;
    //   saveEEPROM();
    // }
    //
    // current_angles[i] += wrapping[i] * 360.0f;

    // --- CLAMP THE TARGET ANGLE ---
    float clamped_target = constrain(target_angles[i], MIN_ANGLE, MAX_ANGLE);

    if (estop_active) clamped_target = current_angles[i];

    // Now, calculate the error to the clamped target. getShortestDelta is not needed
    // because we are no longer on a circle, but on a line segment.
    float error = clamped_target - current_angles[i];

    pid_integral[i] += error * (control_interval / 1000.0f);
    float derivative = (error - pid_last_error[i]) / (control_interval / 1000.0f);
    float output = pidConfig.kp * error + pidConfig.ki * pid_integral[i] + pidConfig.kd * derivative;
    setMotorPWM(i, output*direction[i]);
    pid_last_error[i] = error;
  }
}


void target_angles_callback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
  if (estop_active) return;

  // Fix: Only iterate up to min(NUM_MOTORS, msg->data.size)
  size_t limit = (msg->data.size < NUM_MOTORS) ? msg->data.size : NUM_MOTORS;
  for (size_t i = 0; i < limit; i++) {
    target_angles[i] = (msg->data.data[i]);
  }
}


void estop_callback(const void* msgin)
{
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  estop_active = msg->data;
}


void pid_callback(const void* msgin)
{
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
  pidConfig.kp = msg->data.data[0];
  pidConfig.ki = msg->data.data[1];
  pidConfig.kd = msg->data.data[2];
  
  saveEEPROM(); 
}

void offset_callback(const void* msgin)
{
  const auto* msg =
    (const std_msgs__msg__Float32MultiArray*)msgin;

  // ALWAYS assume ROS may send up to NUM_MOTORS
  size_t count = NUM_MOTORS;

  for (size_t i = 0; i < count; i++)
  {
    if (i < msg->data.size && msg->data.data != NULL)
    {
      offsets[i] = msg->data.data[i];
    }
    else
    {
      offsets[i] = 0.0f;  // safety fallback
    }
  }

  saveEEPROM();
}


void publish_current_angles()
{
  size_t count = NUM_MOTORS;
  for (size_t i = 0; i < count; i++)
  {
    current_angles_msg.data.data[i] = current_angles[i];
  }

  rcl_publish(&current_angles_pub, &current_angles_msg, NULL);
}


void setup() {
  Serial.begin(115200);

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  // Wait for agent
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
    delay(200);
  }

  // Init support and node
  rcl_ret_t rc;
  rc = rclc_support_init(&support, 0, NULL, &allocator);

  rc = rclc_node_init_default(&node, "steering_node", "", &support);

  // Subscriptions
  rc = rclc_subscription_init_default(&target_angles_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/target_angles");

  rc = rclc_subscription_init_default(&estop_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/estop");

  rc = rclc_subscription_init_default(&pid_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pid_params");

  rc = rclc_subscription_init_default(&offsets_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/zero_offsets");

  // Publisher
  rc = rclc_publisher_init_default(&current_angles_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/current_angles");

  // Init messages and backing buffers
  std_msgs__msg__Float32MultiArray__init(&current_angles_msg);
  std_msgs__msg__Float32MultiArray__init(&offset_msg);
  std_msgs__msg__Float32MultiArray__init(&pid_msg);
  std_msgs__msg__Float32MultiArray__init(&target_angles_msg);
  std_msgs__msg__Bool__init(&estop_msg);

  static float current_angles_buf[NUM_MOTORS];
  current_angles_msg.data.data     = current_angles_buf;
  current_angles_msg.data.size     = NUM_MOTORS;
  current_angles_msg.data.capacity = NUM_MOTORS;

  static float offset_buf[NUM_MOTORS];
  offset_msg.data.data     = offset_buf;
  offset_msg.data.size     = NUM_MOTORS;
  offset_msg.data.capacity = NUM_MOTORS;

  static float pid_buf[3];
  pid_msg.data.data     = pid_buf;
  pid_msg.data.size     = 3;
  pid_msg.data.capacity = 3;

  static float target_angles_buf[NUM_MOTORS];
  target_angles_msg.data.data     = target_angles_buf;
  target_angles_msg.data.size     = NUM_MOTORS;
  target_angles_msg.data.capacity = NUM_MOTORS;

  // Executor
  rc = rclc_executor_init(&executor, &support.context, 6, &allocator);

  rc = rclc_executor_add_subscription(&executor, &target_angles_sub, &target_angles_msg,
    &target_angles_callback, ON_NEW_DATA);

  rc = rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
    &estop_callback, ON_NEW_DATA);

  rc = rclc_executor_add_subscription(&executor, &pid_sub, &pid_msg,
    &pid_callback, ON_NEW_DATA);

  rc = rclc_executor_add_subscription(&executor, &offsets_sub, &offset_msg,
    &offset_callback, ON_NEW_DATA);

  // Hardware init
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();

  for (int i = 0; i < NUM_MOTORS; ++i) {
    pinMode(pwmA[i], OUTPUT);
    pinMode(pwmB[i], OUTPUT);
    pinMode(encPins[i], INPUT_PULLDOWN);
  }
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);

  // Initialise the starting Target Angle to the current measure angle.
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(i);
    target_angles[i] = constrain(raw_angle - offsets[i] , MIN_ANGLE, MAX_ANGLE);
  }
}


void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(15));

  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    estop_active = digitalRead(ESTOP_PIN) == LOW;
    controlLoop();
    last_control_time = now;
    publish_current_angles();
  }
}
