// ---- PID constants ----
// Keep your existing R_kp, R_ki, R_kd, L_kp, L_ki, L_kd
// Add a feedforward gain (tune this so robot runs close to target without PID)
double R_kf = 2.5;  // feedforward gain for right motor (PWM per RPM)
double L_kf = 2.5;  // feedforward gain for left motor

int computeRPID(double desiredSpeed, double actualSpeed, double &prevError, double &integral)
{
    if (desiredSpeed == 0) {
        prevError = 0;
        integral = 0;
        return 0;
    }

    // PID error
    double error = desiredSpeed - actualSpeed;
    integral += error;
    double derivative = error - prevError;
    prevError = error;

    // Feedforward (predict PWM needed for desired RPM)
    double ff = R_kf * desiredSpeed;

    // PID correction
    double pid = R_kp * error + R_ki * integral + R_kd * derivative;

    // Combine
    double output = ff + pid;

    // Clamp PWM
    int pwm = abs(output);
    if (pwm > 255) pwm = 255;

    // Minimum PWM to overcome friction
    int minPwm = 30;
    if (pwm < minPwm) pwm = minPwm;

    return (output >= 0) ? pwm : -pwm;
}

int computeLPID(double desiredSpeed, double actualSpeed, double &prevError, double &integral)
{
    if (desiredSpeed == 0) {
        prevError = 0;
        integral = 0;
        return 0;
    }

    double error = desiredSpeed - actualSpeed;
    integral += error;
    double derivative = error - prevError;
    prevError = error;

    double ff = L_kf * desiredSpeed;
    double pid = L_kp * error + L_ki * integral + L_kd * derivative;
    double output = ff + pid;

    int pwm = abs(output);
    if (pwm > 255) pwm = 255;

    int minPwm = 30;
    if (pwm < minPwm) pwm = minPwm;

    return (output >= 0) ? pwm : -pwm;
}
