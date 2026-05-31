#include <Arduino.h>
#include <EEPROM.h>
#include "PwmIn.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32_multi_array.h>


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

const float MAX_ANGLE = 225;
const float MIN_ANGLE = -225;

float offsets[NUM_MOTORS] = {0};
float target_angles[NUM_MOTORS] = {0};
float current_angles[NUM_MOTORS] = {0};
float previous_angles[NUM_MOTORS] = {0};
int wrapping[NUM_MOTORS] = {0};

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


// void saveEEPROM() {
//   EEPROM.put(EEPROM_ADDR, pidConfig);
//   for (int i = 0; i < NUM_MOTORS; ++i)
//     EEPROM.put(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
// }

// void loadEEPROM() {
//   EEPROM.get(EEPROM_ADDR, pidConfig);
//   for (int i = 0; i < NUM_MOTORS; ++i)
//     EEPROM.get(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
//   if (isnan(pidConfig.kp) || pidConfig.kp < 0 || pidConfig.kp > 100)
//     pidConfig = {1.0, 0.0, 0.0};
// }

void saveEEPROM() {
  for (int i = 0; i < NUM_MOTORS; i++)
    EEPROM.put(EEPROM_ADDR + i * sizeof(int), wrapping[i]);
}

void loadEEPROM() {
  for (int i = 0; i < NUM_MOTORS; i++)
    EEPROM.get(EEPROM_ADDR + i * sizeof(int), wrapping[i]);
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

void controlLoop() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(i);
    current_angles[i] = raw_angle - offsets[i] ;
    
    // --- NORMALIZE CURRENT ANGLE to -180 to 180 range ---
    while (current_angles[i] > 180.0f) current_angles[i] -= 360.0f;
    while (current_angles[i] < -180.0f) current_angles[i] += 360.0f;
    
    if (previous_angles[i] > 135.0f and current_angles[i] < -135.0f)
    {
      if (wrapping[i] < 1) wrapping[i] += 1;
      saveEEPROM();
    }
    else if (previous_angles[i] < -135.0f and current_angles[i] > 135.0f)
    {
      if (wrapping[i] > -1) wrapping[i] -= 1;
      saveEEPROM();
    }

    current_angles[i] += wrapping[i] * 360.0f;

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

  // Fix: Only iterate up to min(NUM_JOINTS, msg->data.size)
  size_t limit = (msg->data.size < NUM_MOTORS) ? msg->data.size : NUM_MOTORS;
  for (size_t i = 0; i < NUM_MOTORS; i++) {
    target_angles[i] = (msg->data.data[i]);
  }
}

void publish_current_angles()
{
  current_angles_msg.data.data = current_angles;
  current_angles_msg.data.size = NUM_MOTORS;
  current_angles_msg.data.capacity = NUM_MOTORS;
  rcl_publish(&current_angles_pub, &current_angles_msg, NULL);
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "steering_node", "", &support);

  rclc_subscription_init_default(&target_angles_sub,&node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/target_angles");

  rclc_publisher_init_default(&current_angles_pub,&node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/current_angles");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(&executor, &target_angles_sub, &target_angles_msg,
    &target_angles_callback, ON_NEW_DATA);

  delay(1000);
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
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    estop_active = digitalRead(ESTOP_PIN) == LOW;
    controlLoop();
    last_control_time = now;
    publish_current_angles();
  }
}
