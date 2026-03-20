// ===================== PIN DEFINITIONS =====================
// Motor driver output pins (PWM)
const int motorPinL = D5;
const int motorPinR = D6;

// Sensor input pins (analog)
const int leftSensorPin = A0;
const int rightSensorPin = A1;

// ===================== PID VARIABLES =====================
float previous_error = 0.0;   // Error from previous loop (for derivative term)
float integral = 0.0;         // Accumulated error over time (integral term)
unsigned long last_time = 0;  // Time of previous loop iteration

// =========================================================
// Mouse Project: Line tracking using magnetic field sensors
// Control method: PID with PWM motor control
// =========================================================

void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
}

void loop() {

  // ===================== PID CONSTANTS =====================
  float Kp = 0.4;   // Proportional gain
  float Ki = 0.03;  // Integral gain
  float Kd = 0.05;  // Derivative gain

  // Base motor speed (before correction)
  float base_speed = 50.0;

  // ===================== TIME STEP =====================
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // Time step in seconds

  // ===================== SENSOR READINGS =====================
  float L_sensor_Value = analogRead(leftSensorPin);
  float R_sensor_Value = analogRead(rightSensorPin);

  // Output sensor values for debugging (Serial Plotter)
  Serial.print(L_sensor_Value);
  Serial.print(" ");
  Serial.println(R_sensor_Value);

  // ===================== ERROR CALCULATION =====================
  // Normalisation to prevent bias from signal strength variation
  float normDenom = L_sensor_Value + R_sensor_Value;

  // Prevent division by zero
  if (normDenom == 0) {
    normDenom = 1;
  }

  // Error represents position relative to track centre
  float error = (L_sensor_Value - R_sensor_Value) / normDenom;

  // ===================== PID TERMS =====================
  integral += error * dt;  // Accumulate error over time

  // Rate of change of error (derivative)
  float delta_error = (error - previous_error) / dt;

  // PID correction output
  float correction = Kp * error + Ki * integral + Kd * delta_error;

  // ===================== MOTOR CONTROL =====================
  // Adjust motor speeds to steer left/right
  float leftOutput = base_speed - correction * 255.0;
  float rightOutput = base_speed + correction * 255.0;

  // Constrain outputs to safe PWM range
  leftOutput = constrain(leftOutput, 60, 150);
  rightOutput = constrain(rightOutput, 60, 150);

  // Send PWM signals to motors
  analogWrite(motorPinL, (int)leftOutput);
  analogWrite(motorPinR, (int)rightOutput);

  // ===================== UPDATE STATE =====================
  previous_error = error;
  last_time = current_time;

  // Small delay for stability
  delay(10);
}
