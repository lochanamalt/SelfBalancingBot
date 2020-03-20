//Basic Motion Functions --------------------------------------------
void start()
{
  digitalWrite(rightM_dirP, LOW);
  digitalWrite(rightM_dirN, LOW);
  digitalWrite(leftM_dirP, LOW);
  digitalWrite(leftM_dirN, LOW);
}


//*********************BRAKE **********************
void brake()
{
  digitalWrite(rightM_dirP,HIGH);
  digitalWrite(rightM_dirN,HIGH);
  digitalWrite(leftM_dirP,HIGH);
  digitalWrite(leftM_dirN,HIGH);
}
//*******************   ADJUST SPEED ****************

void motorSpeed(bool state)
{
  analogWrite(leftMEnable, state);
  analogWrite(rightMEnable, state);
}


void motorSpeed(int leftSpeed, int rightSpeed)
{
  analogWrite(rightMEnable,rightSpeed);
  analogWrite(leftMEnable,leftSpeed);
}
//**********************  GO FORWARD MARVIC *************************************

void forward()
{
Lforward();
Rforward();
}

void forward(int LSpeed, int RSpeed)
{
 Lforward();
 Rforward();
 motorSpeed(LSpeed,RSpeed);
}

//********************** REVERSE MARVIC **********************************************

void back()
{
 Lbackward();
 Rbackward();
}

void back(int LSpeed, int RSpeed)
{
 Lbackward();
 Rbackward();
 motorSpeed(LSpeed,RSpeed);
}

//*******************************  TURN LEFT    ***************************************
void leftTurn()
{
 Lbackward();
 Rforward();
}
void leftTurn(int LSpeed, int RSpeed)
{
 Lbackward();
 Rforward();
 motorSpeed(LSpeed,RSpeed);
}

//********************* TURN   RIGHT   *******************************************************************************************
void rightTurn()
{
 Rbackward();
 Lforward();
}

void rightTurn(int LSpeed, int RSpeed)
{
  Rbackward();
  Lforward();
  motorSpeed(LSpeed,RSpeed);
}

//==============MOTOR Directions==================================

void Rforward()
{
  digitalWrite(rightM_dirP,HIGH);
  digitalWrite(rightM_dirN,LOW);
}
void Rbackward()
{
  digitalWrite(rightM_dirP,LOW);
  digitalWrite(rightM_dirN,HIGH);
}
void Lforward()
{
  digitalWrite(leftM_dirP,HIGH);
  digitalWrite(leftM_dirN,LOW);
}
void Lbackward()
{
  digitalWrite(leftM_dirP,LOW);
  digitalWrite(leftM_dirN,HIGH);
}

