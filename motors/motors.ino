#define leftMEnable 5
#define leftM_dirP   7
#define leftM_dirN   8
#define rightMEnable 6
#define rightM_dirP  4
#define rightM_dirN  12

volatile int motorPower;

void setup() {
   // set the motor control and PWM pins to output mode
  pinMode(leftM_dirP, OUTPUT);
  pinMode(leftM_dirN, OUTPUT);
  pinMode(rightM_dirP, OUTPUT);
  pinMode(rightM_dirN, OUTPUT);
  
  pinMode(leftMEnable, OUTPUT);
  pinMode(rightMEnable, OUTPUT);

  digitalWrite(leftMEnable,HIGH);
  digitalWrite(rightMEnable,HIGH);

}

void loop() {
  back(50,50);


}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftM_dirP, leftMotorSpeed);
    digitalWrite(leftM_dirN, LOW);
  }
  else {
    analogWrite(leftM_dirP, 255 + leftMotorSpeed);
    digitalWrite(leftM_dirN, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightM_dirP, rightMotorSpeed);
    digitalWrite(rightM_dirN, LOW);
  }
  else {
    analogWrite(rightM_dirP, 255 + rightMotorSpeed);
    digitalWrite(rightM_dirN, HIGH);
  }
}
