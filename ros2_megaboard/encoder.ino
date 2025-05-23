Encoder leftEnc(left_encoderA, left_encoderB);   // Example encoder pins
Encoder rightEnc(right_encoderA, right_encoderB);

void onRequestEncoder()
{
  left_enc = leftEnc.read();
  right_enc = rightEnc.read();
  cmdMessenger.sendCmdStart(kEncoderData);
  cmdMessenger.sendCmdArg<long>(right_enc);
  cmdMessenger.sendCmdArg<long>(left_enc);
  // cmdMessenger.sendCmdArg<double>(rightDesSpeed);
  // cmdMessenger.sendCmdArg<double>(leftDesSpeed);
  cmdMessenger.sendCmdArg<double>(rightActSpeed);
  cmdMessenger.sendCmdArg<double>(leftActSpeed);
  // cmdMessenger.sendCmdArg<double>(rightPwm);
  // cmdMessenger.sendCmdArg<double>(leftPwm);
  cmdMessenger.sendCmdEnd();
  Serial.println();
}