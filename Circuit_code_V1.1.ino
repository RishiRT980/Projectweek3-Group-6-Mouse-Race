// Define pins to be used by Arduino
const int motorPinL = D5;
const int motorPinR = D6;
const int leftSensorPin = A0;
const int rightSensorPin = A1;
float previous_error = 0.0; 
float integral = 0.0; 
unsigned long last_time = 0;   
 

// Concept taken from:
//Mouse Project involving mouse and sensing, magnetic fields, PWM and PID control

void setup() {
  Serial.begin(9600);
//Setting pin mode operation 
  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
}

void loop() {
  //Define values for kp and set an initial speed value
  float Kp = 0.4;
  float Ki = 0.03;
  float Kd = 0.05; 
  float base_speed = 50.0;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_time)/1000.0; // convert to s
  float L_sensor_Value = analogRead(sensorPinL);
  float R_sensor_Value = analogRead(sensorPinR);
//For sensor diagnostic running on the srial plotter
Serial.print(L_sensor_Value);
Serial.print(" ");
Serial.println(R_sensor_Value);
//Calculation for normalised error
  float normDenom = L_sensor_Value + R_sensor_Value;
//Condition to avoid dividing by zero 
  if (normDenom == 0)
   normDenom = 1;
  float dt = current_time - last_time;
  float error = (L_sensor_Value - R_sensor_Value)/normDenom;
//integral is area under a curve
  float integral += error*dt;
  //? might involve time. might also involve sensor Value.
  float delta_error = (error - previous_error)*dt;
  // need to define previous_error 
  float correction = Kp*error + Ki*integral + Kd*delta_error;
//To enable the mouse to turn left or right
  float leftOutput = base_speed - correction*255.0;
  float rightOutput = base_speed + correction*255.0;
//Keeping speed of wheels within PWM values 60 to 150 
  leftOutput = constrain(leftOutput, 60, 150);
  rightOutput = constrain(rightOutput, 60, 150);
//To keep as integer values 
  analogWrite(motorPinL, (int)leftOutput);
  analogWrite(motorPinR, (int)rightOutput);

//update for looping
  previous_error = error;
  last_time = current_time;
//For stability of code 
  delay(10);
}

//Enable more constants for PID control 
//Integral = area under the curve y dx summed to infinity
