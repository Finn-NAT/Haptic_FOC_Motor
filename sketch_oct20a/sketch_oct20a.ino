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
float target_angle = 0;
uint32_t timer = 0;
bool check = true;

//Serial Command Setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

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
  driver.voltage_power_supply = 15;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver.init();

  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  
  //FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::angle;

  //Speed PID Setting                                     
  motor.PID_velocity.P = 0.1;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor.PID_velocity.I = 0.1;
  //Angle PID Setting 
  motor.P_angle.P = 15;
  motor.P_angle.D = 0.1;
  motor.P_angle.I = 0.01;
  //Motor Maximum Limit Voltage
  motor.voltage_limit = 10;                //According to the supply voltage, modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.01;

  //Maximum Velocity Limit Setting
  motor.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  
  //Initialize the Motor
  motor.init();
  command.add('T', doTarget, "target velocity");
  //Initialize FOC
  motor.sensor_direction = Direction::CCW;
  motor.initFOC();
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}

void loop() {

  float a = wrap_0_2pi(sensor.getAngle());
  Serial.println(a, 4);
  motor.loopFOC();

  motor.move(target_velocity);
  
  command.run();

  // Serial.print(sensor.getAngle()); 
  // Serial.println();
  // motor.loopFOC();
  // // target_angle += 180;
  // // target_velocity = deg2rad(target_angle);

  // if (millis() - timer >= 2000) {
  //   timer = millis();
  //   check = !check;
  //   if(check){
  //     target_velocity += PI;  // +1*pi
  //   }else{
  //     target_velocity -= PI;  // +1*pi
  //   }
    
  // }

  // motor.move(target_velocity);
  // // motor1.move(target_velocity);

  // static uint32_t tSense = 0;
  // if (millis() - tSense >= 1000) {
  //   tSense = millis();
  //   float angle = sensor.getAngle(); // rad
  //   Serial.print("AS5048A: ");
  //   Serial.print(angle, 4);
  //   Serial.println(" rad ");
  // }

  // //User Newsletter
  // // command.run();
  // // _delay(2000);
}
