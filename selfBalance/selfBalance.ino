#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

//------Motor Pins----------------------
// 3,9,10,11 PWM pins will not work when using Timers
#define leftMEnable 5
#define leftM_dirP   7
#define leftM_dirN   8
#define rightMEnable 6
#define rightM_dirP  4
#define rightM_dirN  12


//-------Ultrasonic Pins-----------------
#define TRIGGER_PIN A2
#define ECHO_PIN A1
#define MAX_DISTANCE 75
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


//---------for PID------------------------
#define Kp  70
#define Kd  0.05
#define Ki  40
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot
#define rightMinSpeed -255 // min speed of the robot
#define leftMinSpeed -255 // min speed of the robot


#define sampleTime  0.005
#define targetAngle -0.48

//---Gyro and Accelerometer-----------------
MPU6050 mpu;


volatile int pidMotorSpeed;      //motor speed
int16_t accY, accZ, gyroX;    // accelerometer and gyroscope readings
volatile float accAngle;      // accelerometer angle complementary filter

volatile int gyroRate;      //rotational velocity about x axis
volatile float gyroAngle, currentAngle, prevAngle = 0; //gyro angles for complementary filter
volatile float error, prevError = 0, errorSum = 0; // for PID

volatile byte count = 0;

int distance;   //ultrasonic distance


//*******Timer Interrupt for PID***********************************

void init_PID() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  pinMode(leftM_dirP, OUTPUT);
  pinMode(leftM_dirN, OUTPUT);
  pinMode(rightM_dirP, OUTPUT);
  pinMode(rightM_dirN, OUTPUT);

  pinMode(leftMEnable, OUTPUT);
  pinMode(rightMEnable, OUTPUT);

  digitalWrite(leftMEnable, HIGH);
  digitalWrite(rightMEnable, HIGH);

  // set the status LED to output mode
  pinMode(13, OUTPUT);

  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(382);
  mpu.setZAccelOffset(1448);
  mpu.setXGyroOffset(-6);

  // initialize PID timer
  init_PID();
}

void loop() {

  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();



//    pidMotorSpeed = constrain(pidMotorSpeed, -255, 255);


  int rightMotorSpeed = pidMotorSpeed;
  int leftMotorSpeed = pidMotorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < rightMinSpeed) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < leftMinSpeed) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive
//  Serial.print("angle :");
//  Serial.println(currentAngle);
//  Serial.print("speed :");
//  Serial.println(leftMotorSpeed);
  motorBalance(leftMotorSpeed,rightMotorSpeed);
  

//**************************************************************************

  // measure distance every 100 milliseconds
  if ((count % 20) == 0) {
    distance = sonar.ping_cm();
  }

  //if obstacle is detected, then turn
  if ((distance < 20) && (distance != 0)) {
      motorBalance(-leftMotorSpeed,rightMotorSpeed);
      
  }
}


// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  
  // complementary filter
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  // PID algorithem

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  //calculate output from P, I and D values
  pidMotorSpeed = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;

  count++;
  if (count == 200)  {
    count = 0;
  }
}
