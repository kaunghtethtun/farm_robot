int computeRPID(double desiredSpeed, double actualSpeed, double &prevError, double &integral)
{
  if (desiredSpeed == 0) {
    prevError = 0;
    integral = 0;
    return 0;  // Stop motor: no control needed
  }

  double error = desiredSpeed - actualSpeed;
  integral += error;
  double derivative = error - prevError;
  prevError = error;

  double output = R_kp * error + R_ki * integral + R_kd * derivative;

  // Clamp and preserve direction
  int pwm = abs(output);
  if (pwm > 255) pwm = 255;

  return (output >= 0) ? pwm : -pwm;
}

int computeLPID(double desiredSpeed, double actualSpeed, double &prevError, double &integral)
{
  if (desiredSpeed == 0) {
    prevError = 0;
    integral = 0;
    return 0;  // Stop motor: no control needed
  }

  double error = desiredSpeed - actualSpeed;
  integral += error;
  double derivative = error - prevError;
  prevError = error;

  double output = L_kp * error + L_ki * integral + L_kd * derivative;

  // Clamp and preserve direction
  int pwm = abs(output);
  if (pwm > 255) pwm = 255;

  return (output >= 0) ? pwm : -pwm;
}