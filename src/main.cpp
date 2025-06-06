#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <EEPROM.h>

#define NUM_MOTORS 4
#define EEPROM_ADDR 0

// ---------------- Motor Pin Definitions ----------------
const uint8_t pwmA[NUM_MOTORS] = {2, 4, 6, 8};   // H-Bridge PWM A
const uint8_t pwmB[NUM_MOTORS] = {3, 5, 7, 9};   // H-Bridge PWM B
const uint8_t encPins[NUM_MOTORS] = {10, 11, 12, 13}; // AS5048A PWM

// ---------------- Motor Constants ----------------
const float MAX_ANGLE[NUM_MOTORS] = {90, 90, 90, 90};
const float MIN_ANGLE[NUM_MOTORS] = {-90, -90, -90, -90};
float offsets[NUM_MOTORS] = {0, 0, 0, 0}; // Loaded from EEPROM

// PID tuning values (modifiable via topic)
float Kp = 2.0, Ki = 0.1, Kd = 0.01;
float pid_integral[NUM_MOTORS] = {0};
float pid_last_error[NUM_MOTORS] = {0};

float target_angles[NUM_MOTORS] = {0};
float current_angles[NUM_MOTORS] = {0};

unsigned long last_control_time = 0;
const unsigned long control_interval = 20; // ms

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

// ---------- Helper Functions ---------------

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

float readEncoderPWM(uint8_t pin) {
  // Blocking pulse width read
  uint32_t pulseWidth = pulseIn(pin, HIGH, 3000); // max 3ms
  return ((float)pulseWidth / 2500.0) * 360.0; // Convert to degrees
}

float getShortestDelta(float from, float to) {
  float delta = to - from;
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
  return delta;
}

void controlLoop() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(encPins[i]);
    current_angles[i] = raw_angle - offsets[i];

    float delta = getShortestDelta(current_angles[i], target_angles[i]);
    float target_abs = current_angles[i] + delta;

    // Clamp target if it exceeds limits
    if (target_abs < MIN_ANGLE[i] || target_abs > MAX_ANGLE[i]) {
      delta = (delta > 0) ? delta - 360 : delta + 360;
    }

    float error = delta;
    pid_integral[i] += error * (control_interval / 1000.0);
    float derivative = (error - pid_last_error[i]) / (control_interval / 1000.0);
    float output = Kp * error + Ki * pid_integral[i] + Kd * derivative;
    setMotorPWM(i, output);
    pid_last_error[i] = error;
  }
}

// ---------- ROS Callback Functions ----------

void angle_callback(const void *msgin, void *arg) {
  int idx = *((int *)arg);
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  target_angles[idx] = msg->data;
}

void pid_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 3) {
    Kp = msg->data.data[0];
    Ki = msg->data.data[1];
    Kd = msg->data.data[2];
    EEPROM.put(EEPROM_ADDR, Kp);
    EEPROM.put(EEPROM_ADDR + 4, Ki);
    EEPROM.put(EEPROM_ADDR + 8, Kd);
    EEPROM.commit();
  }
}

void timer_callback(rcl_timer_t *, int) {
  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    controlLoop();
    last_control_time = now;
  }
}

void setupEEPROM() {
  EEPROM.get(EEPROM_ADDR, Kp);
  EEPROM.get(EEPROM_ADDR + 4, Ki);
  EEPROM.get(EEPROM_ADDR + 8, Kd);
  for (int i = 0; i < NUM_MOTORS; ++i) {
    EEPROM.get(EEPROM_ADDR + 12 + i * 4, offsets[i]);
  }
}

// ---------- Setup ----------

void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(pwmA[i], OUTPUT);
    pinMode(pwmB[i], OUTPUT);
    pinMode(encPins[i], INPUT);
  }

  EEPROM.begin(64);
  setupEEPROM();

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "steering_controller", "", &support);

  for (int i = 0; i < NUM_MOTORS; ++i) {
    rclc_subscription_init_default(&subs[i], &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      ("motor" + String(i) + "/target_angle").c_str());
  }

  rclc_subscription_init_default(&pid_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/steering/pid_params");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback);
  rclc_executor_init(&executor, &support.context, NUM_MOTORS + 2, &allocator);

  for (int i = 0; i < NUM_MOTORS; ++i) {
    static int idx[NUM_MOTORS];
    idx[i] = i;
    rclc_executor_add_subscription_with_context(
      &executor, &subs[i], &target_msgs[i], angle_callback, &idx[i], ON_NEW_DATA);
  }

  rclc_executor_add_subscription(&executor, &pid_sub, &pid_msg, &pid_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

// ---------- Loop ----------

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
