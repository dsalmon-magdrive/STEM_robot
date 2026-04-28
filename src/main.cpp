#include <Arduino.h>

#define MOTOR1_EN 2
#define MOTOR1_IN1 4
#define MOTOR1_IN2 5

#define MOTOR2_EN 3
#define MOTOR2_IN1 6
#define MOTOR2_IN2 7

#define MOTOR3_EN 10
#define MOTOR3_IN1 8
#define MOTOR3_IN2 9

void setup() {
  // put your setup code here, to run once:

  delay(500);
  Serial.begin(9600);
  delay(1000);

  Serial.println("Starting...");

  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  pinMode(MOTOR3_EN, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);

  digitalWrite(MOTOR1_EN, HIGH);
  digitalWrite(MOTOR2_EN, HIGH);
  digitalWrite(MOTOR3_EN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Running motor 1...");

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);

  delay(1000);
  
  digitalWrite(MOTOR1_IN1, LOW);

  delay(1000);

  Serial.println("Running motor 2...");

  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);

  delay(1000);

  digitalWrite(MOTOR2_IN1, LOW);

  delay(1000);

  Serial.println("Running motor 3...");

  digitalWrite(MOTOR3_IN1, HIGH);
  digitalWrite(MOTOR3_IN2, LOW);

  delay(1000);

  digitalWrite(MOTOR3_IN1, LOW);

  delay(2000);
}