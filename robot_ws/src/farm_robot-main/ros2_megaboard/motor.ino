void motorSetup()
{
  pinMode(ENA1, OUTPUT);
  pinMode(Right_in1, OUTPUT);
  pinMode(Right_in2, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(Left_in1, OUTPUT);
  pinMode(Left_in2, OUTPUT);
}

void getMotorData(unsigned long time)  {

  rightActSpeed = double((right_enc - countAnt1) * 60000) / double(time * new_enc_ticks);
  leftActSpeed = double((left_enc - countAnt2) * 60000) / double(time * new_enc_ticks);

  countAnt1 = right_enc;
  countAnt2 = left_enc;
}

void onSetVelocity()
{
  rightDesSpeed = cmdMessenger.readInt16Arg();
  leftDesSpeed = cmdMessenger.readInt16Arg();
  cmdMessenger.sendCmd(kAcknowledge, "OK");
}

void updateMotorControl()
{
  int miniPwm = 30;
  leftPwm = computeLPID(leftDesSpeed, leftActSpeed, leftPrevError, leftIntegral);
  rightPwm = computeRPID(rightDesSpeed, rightActSpeed, rightPrevError, rightIntegral);
  // rightPwm = updatePID_R(rightDesSpeed,rightActSpeed);
  // if (leftPwm < miniPwm && leftDesSpeed != 0)
  // {
  //   leftPwm = miniPwm;
  // }
  // if (rightPwm < miniPwm && rightDesSpeed != 0)
  // {
  //   rightPwm = miniPwm;
  // }
  // LEFT motor
  if (leftPwm < 0) {
    digitalWrite(Left_in1, LOW);
    digitalWrite(Left_in2, HIGH);
  } else if (leftPwm > 0) {
    digitalWrite(Left_in1, HIGH);
    digitalWrite(Left_in2, LOW);
  } 
  else
  {
    digitalWrite(Left_in1, LOW);
    digitalWrite(Left_in2, LOW);
  }
  analogWrite(ENA2, abs(leftPwm));
  // RIGHT motor
  if (rightPwm < 0) {
    digitalWrite(Right_in1, LOW);
    digitalWrite(Right_in2, HIGH);
  } else if (rightPwm > 0) {
    digitalWrite(Right_in1, HIGH);
    digitalWrite(Right_in2, LOW);
  } 
  else
  {
    digitalWrite(Right_in1, LOW);
    digitalWrite(Right_in2, LOW);
  }
  
  analogWrite(ENA1, abs(rightPwm));
}