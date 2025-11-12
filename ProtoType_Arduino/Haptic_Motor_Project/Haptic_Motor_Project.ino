#include <SimpleFOC.h>
#include <SPI.h>
#include "Motor_Control_Haptic.h"

Commander command = Commander(Serial);

// void doTargetp(char* cmd) { command.scalar(&kp, cmd);  motor.P_angle.P = kp;}
// void doTargeti(char* cmd) { command.scalar(&ki, cmd); motor.P_angle.I = ki;}
// void doTargetd(char* cmd) { command.scalar(&kd, cmd); motor.P_angle.D = kd;}

void setup() {

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  
  // command.add('P', doTargetp, "target P");
  // command.add('I', doTargeti, "target I");
  // command.add('D', doTargetd, "target D");

  MotorHaptic_Init();
  MotorHaptic_Calib();
}

void loop() {
  MotorHaptic_Loop();
}
