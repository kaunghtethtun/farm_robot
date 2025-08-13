#include <CmdMessenger.h>
#include <Encoder.h>
#include "variables.h"

// Serial communication
CmdMessenger cmdMessenger(Serial);

void attachCallbacks()
{
  cmdMessenger.attach(kSetVelocity, onSetVelocity);
  //cmdMessenger.attach(kRequestEncoder, onRequestEncoder);
}

void setup()
{
  Serial.begin(115200);
  motorSetup();
  attachCallbacks();
}

void loop()
{
  unsigned long now = millis();
  
  if (now - lastUpdateTime >= 50) {  // 100ms interval
    getMotorData(now - lastUpdateTime);
    updateMotorControl();
    onRequestEncoder();
    lastUpdateTime = now;
  }
  cmdMessenger.feedinSerialData();
  //onRequestEncoder();
}
