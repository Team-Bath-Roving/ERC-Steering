#include <Arduino.h>
#include <EEPROM.h>
#include "PwmIn.h"

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

void saveEEPROM() {
//   EEPROM.put(EEPROM_ADDR, pidConfig);
//   for (int i = 0; i < NUM_MOTORS; ++i)
//     EEPROM.put(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
}

void loadEEPROM() {
//   EEPROM.get(EEPROM_ADDR, pidConfig);
//   for (int i = 0; i < NUM_MOTORS; ++i)
//     EEPROM.get(EEPROM_ADDR + sizeof(pidConfig) + i * sizeof(float), offsets[i]);
//   if (isnan(pidConfig.kp) || pidConfig.kp < 0 || pidConfig.kp > 100)
//     pidConfig = {1.0, 0.0, 0.0};
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

// void controlLoop() {
//   for (int i = 0; i < NUM_MOTORS; ++i) {
//     // setMotorPWM(i, 255);
//     float raw_angle = readEncoderPWM(i);
//     current_angles[i] = raw_angle - offsets[i];

//     if (estop_active) target_angles[i] = current_angles[i];

//     float delta = getShortestDelta(current_angles[i], target_angles[i]);
//     float target_abs = current_angles[i] + delta;
//     if (target_abs < MIN_ANGLE || target_abs > MAX_ANGLE)
//       delta = (delta > 0) ? delta - 360 : delta + 360;

//     float error = delta;
//     pid_integral[i] += error * (control_interval / 1000.0f);
//     float derivative = (error - pid_last_error[i]) / (control_interval / 1000.0f);
//     float output = pidConfig.kp * error + pidConfig.ki * pid_integral[i] + pidConfig.kd * derivative;
//     setMotorPWM(i, output);
//     pid_last_error[i] = error;
//   }
// }

void controlLoop() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    float raw_angle = readEncoderPWM(i);
    current_angles[i] = raw_angle - offsets[i];

    // --- NORMALIZE CURRENT ANGLE to -180 to 180 range ---
    while (current_angles[i] > 180.0f) current_angles[i] -= 360.0f;
    while (current_angles[i] < -180.0f) current_angles[i] += 360.0f;

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

// void controlLoop() {
//   for (int i = 0; i < NUM_MOTORS; ++i) {
//     float raw_angle = readEncoderPWM(i);
//     current_angles[i] = raw_angle - offsets[i];

//     if (estop_active) target_angles[i] = current_angles[i];

//     // --- CORRECTED LOGIC ---
//     // 1. Calculate the shortest path.
//     float delta = getShortestDelta(current_angles[i], target_angles[i]);

//     // 2. The error for the PID is simply this delta.
//     float error = delta;
    
//     // (The problematic "if (target_abs...)" block has been completely removed)

//     pid_integral[i] += error * (control_interval / 1000.0f);
//     float derivative = (error - pid_last_error[i]) / (control_interval / 1000.0f);
//     float output = pidConfig.kp * error + pidConfig.ki * pid_integral[i] + pidConfig.kd * derivative;
//     setMotorPWM(i, output*direction[i]);
//     pid_last_error[i] = error;
//   }
// }

void parseSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("A")) {
      sscanf(cmd.c_str(), "A %f %f %f %f", &target_angles[0], &target_angles[1], &target_angles[2], &target_angles[3]);
      Serial.println("Target angles set.");
    }
    else if (cmd.startsWith("P")) {
      sscanf(cmd.c_str(), "P %f %f %f", &pidConfig.kp, &pidConfig.ki, &pidConfig.kd);
      saveEEPROM();
      Serial.println("PID parameters updated.");
    }
    else if (cmd.startsWith("O")) {
      sscanf(cmd.c_str(), "O %f %f %f %f", &offsets[0], &offsets[1], &offsets[2], &offsets[3]);
      saveEEPROM();
      Serial.println("Offsets set.");
    }
    else if (cmd.startsWith("R")) {
      Serial.print("Angles: ");
      for (int i = 0; i < NUM_MOTORS; ++i) {
        Serial.print(current_angles[i]);
        Serial.print(i < NUM_MOTORS - 1 ? ", " : "\n");
      }
    }
    else {
      Serial.println("Unknown command. Use A/P/O/R.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
//   EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();

  for (int i = 0; i < NUM_MOTORS; ++i) {
    pinMode(pwmA[i], OUTPUT);
    pinMode(pwmB[i], OUTPUT);
    pinMode(encPins[i], INPUT_PULLDOWN);
  }
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  Serial.println("Ready. Send A/P/O/R commands.");
}

void loop() {

  parseSerial();
//   for (int i = 0; i < NUM_MOTORS; ++i) {
//     setMotorPWM(i,1024);
//     }
//     delay(1000);
// for (int i = 0; i < NUM_MOTORS; ++i) {
//     setMotorPWM(i,-1024);
//     }
//     delay(1000);
  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    estop_active = digitalRead(ESTOP_PIN) == LOW;
    controlLoop();
    last_control_time = now;
    Serial.print("Angles: ");
    for (int i = 0; i < NUM_MOTORS; ++i) {
        Serial.print(current_angles[i]);
        Serial.print(i < NUM_MOTORS - 1 ? ", " : "\n");
        }
  }
}
