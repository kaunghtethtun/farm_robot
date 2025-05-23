// Motor pins
#define ENA1 10
#define Right_in1 9
#define Right_in2 8
#define ENA2 5
#define Left_in1 7
#define Left_in2 6

#define right_encoderA      2     // (interrupt numbers 1)
#define right_encoderB      3
#define left_encoderA      18     // (interrupt numbers 5)
#define left_encoderB      19     // (interrupt numbers 4)

#define new_enc_ticks   940


int leftActSpeed = 0;
int rightActSpeed = 0;
int leftDesSpeed = 0;
int rightDesSpeed = 0;
int leftPwm = 0;
int rightPwm = 0;


long left_enc = 0;
long right_enc = 0;
long countAnt1 = 0;
long countAnt2 = 0; 

unsigned long lastUpdateTime = 0;

double leftPrevError = 0, leftIntegral = 0;
double rightPrevError = 0, rightIntegral = 0;

double R_kp = 2.0;
double R_ki = 0.05;
double R_kd = 0.001;

double L_kp = 1.99;
double L_ki = 0.05;
double L_kd = 0.001;

enum
{
  kSetVelocity = 1,
  kRequestEncoder = 2,
  kEncoderData = 3,
  kAcknowledge = 4,
};
