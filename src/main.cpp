#include <Arduino.h>

#define MOTOR1_EN 4
#define MOTOR1_IN1 2
#define MOTOR1_IN2 3

#define MOTOR2_EN 7
#define MOTOR2_IN1 5
#define MOTOR2_IN2 6

#define MOTOR3_EN 8
#define MOTOR3_IN1 9
#define MOTOR3_IN2 10

#define DEBUG 1

struct motion_vector {
  float angle_degrees;
  float speed; // Speed (m/s) - adjust as needed for your robot's capabilities
} motion_vector;

float mass = 2.0; // Mass of the robot (kg) - adjust as needed for more accurate physics calculations

void set_motion(int bearing_degrees, float speed) {

  // Angle offsets for wheels on each corner of the robot
  float theta_A = bearing_degrees + 180.0;
  float theta_B = bearing_degrees + 60.0;
  float theta_C = bearing_degrees - 60.0;

  // Calculate motor speeds based on the desired bearing and speed
  float motor1_speed = (speed * (float)255) * sin(theta_A * (PI / 180.0));
  float motor2_speed = (speed * (float)255) * sin(theta_B * (PI / 180.0));
  float motor3_speed = (speed * (float)255) * sin(theta_C * (PI / 180.0));

  // Convert motor speeds to integers for PWM output
  int motor1_speed_int = (int) round(motor1_speed);
  int motor2_speed_int = (int) round(motor2_speed);
  int motor3_speed_int = (int) round(motor3_speed);

  // Debugging output to Serial Monitor
  #if DEBUG
  Serial.println("Calculated motor speeds:");
  Serial.print("Motor 1: ");
  Serial.println(motor1_speed);
  Serial.print("Motor 1 as int: ");
  Serial.println(motor1_speed_int);

  Serial.print("Motor 2: ");
  Serial.println(motor2_speed);
  Serial.print("Motor 2 as int: ");
  Serial.println(motor2_speed_int);

  Serial.print("Motor 3: ");
  Serial.println(motor3_speed);
  Serial.print("Motor 3 as int: ");
  Serial.println(motor3_speed_int);
  #endif

  // Set motor directions and speeds based on the calculated values
  if (motor1_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 1 forward");
    #endif
    digitalWrite(MOTOR1_EN, HIGH);
    analogWrite(MOTOR1_IN1, motor1_speed_int);
    analogWrite(MOTOR1_IN2, 0);
  } else if (motor1_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 1 backward");
    #endif
    digitalWrite(MOTOR1_EN, HIGH);
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, abs(motor1_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 1 stopped");
    #endif
    digitalWrite(MOTOR1_EN, LOW);
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, 0);
  }

  if (motor2_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 2 forward");
    #endif
    digitalWrite(MOTOR2_EN, HIGH);
    analogWrite(MOTOR2_IN1, motor2_speed_int);
    analogWrite(MOTOR2_IN2, 0);
  } else if (motor2_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 2 backward");
    #endif
    digitalWrite(MOTOR2_EN, HIGH);
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, abs(motor2_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 2 stopped");
    #endif
    digitalWrite(MOTOR2_EN, LOW);
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, 0);
  }

  if (motor3_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 3 forward");
    #endif
    digitalWrite(MOTOR3_EN, HIGH);
    analogWrite(MOTOR3_IN1, motor3_speed_int);
    analogWrite(MOTOR3_IN2, 0);
  } else if (motor3_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 3 backward");
    #endif
    digitalWrite(MOTOR3_EN, HIGH);
    analogWrite(MOTOR3_IN1, 0);
    analogWrite(MOTOR3_IN2, abs(motor3_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 3 stopped");
    #endif
    digitalWrite(MOTOR3_EN, LOW);
    analogWrite(MOTOR3_IN1, 0);
    analogWrite(MOTOR3_IN2, 0);
  }
}

void apply_impulse(float impulse_magnitude, float impulse_angle_degrees) {
  // Calculate the angle of the desired motion vector

  #if DEBUG
  Serial.println("Applying impulse:");
  Serial.print("Impulse magnitude: ");
  Serial.println(impulse_magnitude);
  Serial.print("Impulse angle: ");
  Serial.println(impulse_angle_degrees);
  #endif

  float speed_x = motion_vector.speed * cos(motion_vector.angle_degrees * (PI / 180.0));
  float speed_y = motion_vector.speed * sin(motion_vector.angle_degrees * (PI / 180.0));

  #if DEBUG
  Serial.println("Existing motion scalars:");
  Serial.print("X: ");
  Serial.println(speed_x);
  Serial.print("Y: ");
  Serial.println(speed_y);
  #endif

  float impulse_x = impulse_magnitude * cos(impulse_angle_degrees * (PI / 180.0));
  float impulse_y = impulse_magnitude * sin(impulse_angle_degrees * (PI / 180.0));

  #if DEBUG
  Serial.println("Applied impulse scalars:");
  Serial.print("X: ");
  Serial.println(impulse_x);
  Serial.print("Y: ");
  Serial.println(impulse_y);
  #endif

  float new_speed_x = speed_x + (impulse_x / mass);
  float new_speed_y = speed_y + (impulse_y / mass);

  #if DEBUG
  Serial.println("New motion scalars:");
  Serial.print("X: ");
  Serial.println(new_speed_x);
  Serial.print("Y: ");
  Serial.println(new_speed_y);
  #endif

  float new_speed = sqrt(new_speed_x * new_speed_x + new_speed_y * new_speed_y);
  float new_angle_degrees = atan2(new_speed_y, new_speed_x) * (180.0 / PI);

  #if DEBUG
  Serial.println("New motion vector:");
  Serial.print("Speed: ");
  Serial.println(new_speed);
  Serial.print("Angle: ");
  Serial.println(new_angle_degrees);
  #endif

  motion_vector.angle_degrees = new_angle_degrees;
  motion_vector.speed = new_speed;

  #if DEBUG
  Serial.println("Setting new motion vector:");
  Serial.print("Speed: ");
  Serial.println(motion_vector.speed);
  Serial.print("Angle: ");
  Serial.println(motion_vector.angle_degrees);
  #endif

  set_motion(new_angle_degrees, new_speed);

}

void setup() {

  // Initialize Serial communication for debugging
  delay(1000);
  Serial.begin(9600);
  delay(1000);

  #if DEBUG
  Serial.println("Debug mode enabled");
  Serial.println("Starting...");
  #endif

  // Set motor control pins as outputs and initialize them to LOW
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  pinMode(MOTOR3_EN, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);

  #if DEBUG
  Serial.println("Initializing motor control pins...");
  #endif

  digitalWrite(MOTOR1_EN, LOW);
  digitalWrite(MOTOR2_EN, LOW);
  digitalWrite(MOTOR3_EN, LOW);

  motion_vector.angle_degrees = 0.0;
  motion_vector.speed = 0.0;

  #if DEBUG
  Serial.println("Initialised motion vector:");
  Serial.print("Speed: ");
  Serial.println(motion_vector.speed);
  Serial.print("Angle: ");
  Serial.println(motion_vector.angle_degrees);
  #endif

  Serial.println("----------");
  apply_impulse(1.0, 45.0); // Example impulse of magnitude 1.0 at a 45 degree angle
  delay(1000);
  Serial.println("----------");
  apply_impulse(-1.0, 45.0); // Example impulse of magnitude 1.0 at a 135 degree angle
  delay(1000);
  Serial.println("----------");
  apply_impulse(1.0, 180.0); // Example impulse of magnitude 1.0 at a 180 degree angle
  delay(1000);
  Serial.println("----------");
  apply_impulse(1.0, 0.0); // Example impulse of magnitude 1.0 at a 0 degree angle
  delay(1000);

}

void loop() {




  // Test the set_motor_speeds function with different angles and a fixed speed
  // float speed = 0.5;
  // int delay_time = 1000;

  // Serial.println("Angle = 0");
  // set_motion(0, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = 30");
  // set_motion(30, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = 60");
  // set_motion(60, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = -30");
  // set_motion(-30, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = -60");
  // set_motion(-60, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = 180");
  // set_motion(180, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

  // Serial.println("Angle = 270");
  // set_motion(270, speed);
  // delay(delay_time);
  // set_motion(0, 0);
  // delay(delay_time*2);

}