double R_kf = 2.5; 
double L_kf = 2.5; 

int computeRPID(double desiredSpeed, double actualSpeed, double &prevError, double &integral)
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
   
    double ff = R_kf * desiredSpeed;
   
    double pid = R_kp * error + R_ki * integral + R_kd * derivative;
    
    double output = ff + pid;
   
    int pwm = abs(output);
    if (pwm > 255) pwm = 255;

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
