// MKS ESP32 FOC Open Loop Position Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
// Enter "T+number" in the serial port to set the position of the two motors.
// For example, enter "T3.14" (180 degrees in radians) to make the motor rotate to the 180 degree position.
// When using your own motor, please remember to modify the default number of pole pairs, the value in BLDCMotor()
// The default power supply voltage set by the program is 12V.
// Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply

#include <SimpleFOC.h>
#include <SPI.h>

BLDCMotor motor = BLDCMotor(7);         //According to the selected motor, modify the value in BLDCMotor()
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

// Cảm biến AS5048A qua SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

//Target Variable
float target_velocity = 0;
float target_angle = 3.14;
float target_angle2 = 3.14;
uint32_t timer = 0;
bool check = false;

float kp = 140;
float ki = 0.005;
float kd = 0.004;

float kpv = 0.5;
float kiv = 0.1;

//Serial Command Setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void doTargetp(char* cmd) { command.scalar(&kp, cmd);  motor.P_angle.P = kp;}
void doTargeti(char* cmd) { command.scalar(&ki, cmd); motor.P_angle.I = ki;}
void doTargetd(char* cmd) { command.scalar(&kd, cmd); motor.P_angle.D = kd;}

void doTargetpv(char* cmd) { command.scalar(&kpv, cmd); motor.PID_velocity.P = kpv;}
void doTargetiv(char* cmd) { command.scalar(&kiv, cmd); motor.PID_velocity.I = kiv;}

static inline float wrap_0_2pi(float a) {
  a = fmodf(a, _2PI);
  if (a < 0) a += _2PI;
  return a;
}

float deg2rad(float deg);

void setup() {

  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  // sensor.spi_mode = SPI_MODE0;
  sensor.clock_speed = 50000;
  sensor.init(&spiBus);
  //Connect the Motor Object with the Sensor Object
  motor.linkSensor(&sensor);

  //Supply Voltage Setting [V]
  driver.voltage_power_supply = 20;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver.init();

  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  
  //FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  //motor.controller = MotionControlType::angle;
  motor.controller = MotionControlType::torque;

  //Speed PID Setting                                     
  motor.PID_velocity.P = kpv;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor.PID_velocity.I = kiv;
  //Angle PID Setting 
  motor.P_angle.P = kp;
  motor.P_angle.D = kd;
  motor.P_angle.I = ki;
  //Motor Maximum Limit Voltage
  motor.voltage_limit = 15;                //According to the supply voltage, modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.04;

  //Maximum Velocity Limit Setting
  motor.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  
  //Initialize the Motor
  motor.init();
  command.add('T', doTarget, "target angle");
  
  command.add('P', doTargetp, "target P");
  command.add('I', doTargeti, "target I");
  command.add('D', doTargetd, "target D");

  command.add('PV', doTargetp, "target PV");
  command.add('IV', doTargeti, "target IV");
  //Initialize FOC
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 2.783409;
  motor.initFOC();
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  motor.loopFOC();
  target_angle = motor.shaft_angle;
  target_angle2 = motor.shaft_angle + 1;
  motor.move(target_angle);
}

float angle_dead_zone = 4 * PI / 180.0;
float angle_dead_zone2 = 5 * PI / 180.0;
uint32_t start_time = 0;
bool ot = false;
bool ot2 = false;

void loop() {

  motor.loopFOC();

  // float error = fabs(target_angle - motor.shaft_angle);
  // float error2 = fabs(target_angle2 - motor.shaft_angle);
  // float required_torque_input = motor.voltage.q;
   
  // if (error < angle_dead_zone) {
  //   angle_dead_zone  = 7 * PI / 180.0;
  //   motor.P_angle.P = kp;
  //   //Serial.println(a, 4);
  //   motor.controller = MotionControlType::angle;
  //   motor.P_angle.P = kp;
  //   motor.move(target_angle);
  //   Serial.println(error*RAD_TO_DEG);
  // }
  // else if(error2 < angle_dead_zone2 && check){
  //   motor.P_angle.P = 100;
  //   motor.controller = MotionControlType::angle;
  //   motor.P_angle.P = 100;
  //   motor.move(target_angle2);
  //   Serial.println(required_torque_input);    
  // }
  // else {
  //     angle_dead_zone = 1 * PI / 180.0;
  //     check = false;
  //     Serial.println(motor.shaft_velocity);    
  //     if(fabs(motor.shaft_velocity) < 0.05 ){
  //       start_time++;        
  //     }
  //     if(start_time > 500){
  //       target_angle2 = motor.shaft_angle;
  //       check = true;
  //       start_time = 0;
  //     }
  //     motor.controller = MotionControlType::torque;
  //     motor.move(0);
  // }

  
  command.run();

  // float error = -target_angle + motor.shaft_angle;
  // float error2 = -target_angle2 + motor.shaft_angle;
  // float vec = motor.shaft_velocity;
   
  // if (fabs(error) < angle_dead_zone) {
  //   angle_dead_zone  = 7 * PI / 180.0;
  //   motor.controller = MotionControlType::angle;
  //   motor.move(target_angle);
  //   Serial.println(motor.voltage.q);
  //   // angle_dead_zone  = 7 * PI / 180.0;
  //   // float torque_T = - kp * error  - kd * vec;
  //   // motor.move(torque_T);
  //   // Serial.println(error*RAD_TO_DEG);
  // }
  // // else if(fabs(error2) < angle_dead_zone2 && check){
  // //   motor.controller = MotionControlType::torque;
  // //   float torque_T = - 50 * error2  - kd * vec;
  // //   motor.move(torque_T);
  // //   Serial.println(error*RAD_TO_DEG);    
  // // }
  // else {
  //     angle_dead_zone = 1 * PI / 180.0;
  //     check = false;
  //     //Serial.println(motor.shaft_velocity);    
  //     if(fabs(motor.shaft_velocity) < 0.05 ){
  //       start_time++;        
  //     }
  //     if(start_time > 500){
  //       target_angle2 = motor.shaft_angle;
  //       check = true;
  //       start_time = 0;
  //     }
  //     motor.controller = MotionControlType::torque;
  //     motor.move(0);
  // }

  float error = fabs(target_angle - motor.shaft_angle);
  float error2 = fabs(target_angle2 - motor.shaft_angle);
  float required_torque_input = motor.voltage.q;
   
  if (error < angle_dead_zone) {
    angle_dead_zone  = 7 * PI / 180.0;
    if(ot){
    motor.move(12);
    motor.loopFOC();
    delayMicroseconds(500);
    motor.move(10);
    motor.loopFOC();
    delayMicroseconds(500);
    motor.P_angle.P = kp;
    motor.P_angle.reset();
    motor.PID_velocity.reset();
    //Serial.println(a, 4);
    motor.controller = MotionControlType::angle;
    ot = false;
    }
    motor.move(target_angle);
    Serial.println(error*RAD_TO_DEG);
  }
  else if(error2 < angle_dead_zone2 && check){
    if(ot){
    motor.P_angle.P = 60;
    motor.P_angle.reset();
    motor.PID_velocity.reset();
    motor.controller = MotionControlType::angle;
    ot = false;
    }
    motor.move(target_angle2);
    Serial.println(required_torque_input);    
  }
  else {
    ot = true;
      angle_dead_zone = 1.5 * PI / 180.0;
      check = false;
      Serial.println(motor.shaft_velocity);    
      if(fabs(motor.shaft_velocity) < 0.05 ){
        start_time++;        
      }
      if(start_time > 500){
        target_angle2 = motor.shaft_angle;
        check = true;
        start_time = 0;
      }
      motor.controller = MotionControlType::torque;
      motor.move(0);
  }

  delayMicroseconds(1000);
}
