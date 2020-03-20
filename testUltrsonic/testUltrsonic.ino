#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

//-------Ultrasonic Pins-----------------
#define TRIGGER_PIN A2
#define ECHO_PIN A1
#define MAX_DISTANCE 75
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int FRightDistance;
int FRightDuration;
int distanceCm;   //ultrasonic distance
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
//  pinMode(TRIGGER_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);
}

void loop() {
 distanceCm = sonar.ping_cm();
 Serial.println(distanceCm);
// Serial.println(FRightUltrasonic());

}

unsigned int FRightUltrasonic() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  FRightDuration = pulseIn(ECHO_PIN, HIGH);
  return (FRightDuration / 2) / 29.1;
}
