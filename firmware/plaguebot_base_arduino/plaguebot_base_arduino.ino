/*
 * Plague-Bot VR - Base Motor Controller
 * ESP32-C6 + 4x IBT-2 Motor Drivers
 * 
 * Serial protocol (compatible with bumperbot_firmware hardware interface):
 *   Receive: rp05.50,lp05.50,  (r=right, l=left, p/n=direction, value=rad/s)
 *   Send:    rp00.00,lp00.00,  (measured velocities from encoders)
 *
 * Motor layout (viewed from behind):
 *   Left:  M1 (front), M3 (rear)
 *   Right: M2 (front), M4 (rear)
 *
 * IBT-2 control: RPWM = forward, LPWM = reverse
 */

// ================= Motor Pin Mapping =================
// Left side
const int M1_RPWM = 0;
const int M1_LPWM = 1;
const int M3_RPWM = 6;
const int M3_LPWM = 7;
// Right side
const int M2_RPWM = 4;
const int M2_LPWM = 5;
const int M4_RPWM = 2;
const int M4_LPWM = 23;

// ================= Encoder Pin Mapping (for future use) =================
// TODO: Define when encoders are physically connected
// const int RIGHT_ENC_A = ??;
// const int RIGHT_ENC_B = ??;
// const int LEFT_ENC_A  = ??;
// const int LEFT_ENC_B  = ??;

// ================= Encoder Variables =================
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";
String left_wheel_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;  // 10 Hz

// ================= Serial Command Parsing =================
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// ================= Motor Commands =================
double right_wheel_cmd_vel = 0.0;  // rad/s from ROS
double left_wheel_cmd_vel = 0.0;   // rad/s from ROS
double right_wheel_meas_vel = 0.0; // rad/s measured
double left_wheel_meas_vel = 0.0;  // rad/s measured

// ================= PWM Config =================
const int PWM_FREQ = 5000;
const int PWM_RES = 8;  // 0-255
const double MAX_RAD_S = 7.0;  // rad/s at PWM 255, tune to your robot

void setup() {
  Serial.begin(115200);

  // Configure all motor pins as PWM outputs
  int motorPins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM,
                     M3_RPWM, M3_LPWM, M4_RPWM, M4_LPWM};
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  // TODO: Enable when encoders are connected
  // pinMode(RIGHT_ENC_B, INPUT);
  // pinMode(LEFT_ENC_B, INPUT);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderCallback, RISING);
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderCallback, RISING);
}

// ================= Motor Control =================
void setLeftMotors(double cmd_vel) {
  int pwm = (int)constrain(abs(cmd_vel) / MAX_RAD_S * 255.0, 0, 255);
  if (cmd_vel >= 0) {
    analogWrite(M1_RPWM, pwm);  analogWrite(M1_LPWM, 0);
    analogWrite(M3_RPWM, pwm);  analogWrite(M3_LPWM, 0);
  } else {
    analogWrite(M1_RPWM, 0);    analogWrite(M1_LPWM, pwm);
    analogWrite(M3_RPWM, 0);    analogWrite(M3_LPWM, pwm);
  }
}

void setRightMotors(double cmd_vel) {
  int pwm = (int)constrain(abs(cmd_vel) / MAX_RAD_S * 255.0, 0, 255);
  if (cmd_vel >= 0) {
    analogWrite(M2_RPWM, pwm);  analogWrite(M2_LPWM, 0);
    analogWrite(M4_RPWM, pwm);  analogWrite(M4_LPWM, 0);
  } else {
    analogWrite(M2_RPWM, 0);    analogWrite(M2_LPWM, pwm);
    analogWrite(M4_RPWM, 0);    analogWrite(M4_LPWM, pwm);
  }
}

void stopMotors() {
  int motorPins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM,
                     M3_RPWM, M3_LPWM, M4_RPWM, M4_LPWM};
  for (int i = 0; i < 8; i++) {
    analogWrite(motorPins[i], 0);
  }
}

// ================= Main Loop =================
void loop() {
  // Parse serial commands (same protocol as Bumper-Bot)
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p') {
      // Positive direction — no pin toggle needed for IBT-2
      // Direction is handled by which pin gets PWM
    }
    else if (chr == 'n') {
      // Negative direction — handled in setLeftMotors/setRightMotors
    }
    else if (chr == ',') {
      double vel = atof(value);
      if (is_right_wheel_cmd) {
        // Apply sign based on last direction char
        right_wheel_cmd_vel = vel;
      } else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = vel;
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = '0'; value[1] = '0'; value[2] = '.';
      value[3] = '0'; value[4] = '0'; value[5] = '\0';
    }
    else {
      if (value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Periodic: read encoders, send feedback, apply motor commands
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    // TODO: Calculate real velocities when encoders are connected
    // For now, echo commanded velocities as measured (open loop)
    right_wheel_meas_vel = right_wheel_cmd_vel;
    left_wheel_meas_vel = left_wheel_cmd_vel;

    // Send encoder feedback (same format as Bumper-Bot)
    char right_sign = right_wheel_cmd_vel >= 0 ? 'p' : 'n';
    char left_sign = left_wheel_cmd_vel >= 0 ? 'p' : 'n';
    String encoder_read = "r" + String(right_sign) + String(abs(right_wheel_meas_vel))
                        + ",l" + String(left_sign) + String(abs(left_wheel_meas_vel)) + ",";
    Serial.println(encoder_read);

    // Apply motor commands
    if (is_cmd_complete) {
      setRightMotors(right_wheel_cmd_vel);
      setLeftMotors(left_wheel_cmd_vel);
    }

    // Safety: stop if no command received
    if (right_wheel_cmd_vel == 0.0 && left_wheel_cmd_vel == 0.0) {
      stopMotors();
    }

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
  }
}

// ================= Encoder Callbacks (for future use) =================
void rightEncoderCallback() {
  // TODO: Enable when encoders are connected
  // if (digitalRead(RIGHT_ENC_B) == HIGH) {
  //   right_wheel_sign = "p";
  // } else {
  //   right_wheel_sign = "n";
  // }
  right_encoder_counter++;
}

void leftEncoderCallback() {
  // TODO: Enable when encoders are connected
  // if (digitalRead(LEFT_ENC_B) == HIGH) {
  //   left_wheel_sign = "n";
  // } else {
  //   left_wheel_sign = "p";
  // }
  left_encoder_counter++;
}
