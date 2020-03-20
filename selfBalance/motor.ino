void motorBalance(int leftMotorSpeed,int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
//    leftMotorSpeed = constrain(leftMotorSpeed, 100, 255);
    analogWrite(leftMEnable, leftMotorSpeed);
    Lforward();
  }
  else {
//    Serial.print("motor left negative :");
//    Serial.print("      ");
//    Serial.print(leftMotorSpeed);
//    Serial.println(-1 * leftMotorSpeed);
//    leftMotorSpeed = constrain(leftMotorSpeed, -255, -100);
    analogWrite(leftMEnable, -1 * leftMotorSpeed);
    Lbackward();
  }
  if(rightMotorSpeed >= 0) {
//    rightMotorSpeed = constrain(rightMotorSpeed, 100, 255);
    analogWrite(rightMEnable, rightMotorSpeed);
    Rforward();
  }
  else {

//     Serial.print("motor right negative :");
//      Serial.print("      ");
//    Serial.print(rightMotorSpeed);
//    Serial.println(-1 *rightMotorSpeed);
//   rightMotorSpeed = constrain(rightMotorSpeed, -255, -100);
    analogWrite(rightMEnable,-1 *rightMotorSpeed);
    Rbackward();
  }
}
