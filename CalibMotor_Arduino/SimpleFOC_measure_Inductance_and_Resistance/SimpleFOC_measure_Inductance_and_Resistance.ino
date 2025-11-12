#include <SimpleFOC.h>
#include <SPI.h>

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);

BLDCMotor motor = BLDCMotor(7);         //According to the selected motor, modify the value in BLDCMotor()
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

// characterisation voltage set point variable
float characteriseVolts = 0.0f;

// instantiate the commander
Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}
void characterise(char* cmd) { 
  command.scalar(&characteriseVolts, cmd); 
  motor.characteriseMotor(characteriseVolts);
}


void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  // sensor.spi_mode = SPI_MODE0;
  sensor.clock_speed = 50000;
  sensor.init(&spiBus);
  //Connect the Motor Object with the Sensor Object
  motor.linkSensor(&sensor);

  //Supply Voltage Setting [V]
  driver.voltage_power_supply = 12;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // set torque mode:
  // TorqueControlType::dc_current
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::foc_current;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // foc current control parameters (Arduino UNO/Mega)
  // motor.PID_current_q.P = 5;
  // motor.PID_current_q.I= 300;
  // motor.PID_current_d.P= 5;
  // motor.PID_current_d.I = 300;
  // motor.LPF_current_q.Tf = 0.01f;
  // motor.LPF_current_d.Tf = 0.01f;
  // foc current control parameters (stm/esp/due/teensy)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 1000;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add commands M & L
  command.add('M',&onMotor,"Control motor");
  command.add('L', characterise, "Characterise motor L & R with the given voltage");

  motor.disable();

  Serial.println(F("Motor disabled and ready."));
  Serial.println(F("Control the motor and measure the inductance using the terminal. Type \"?\" for available commands:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or torque (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move();

  // user communication
  command.run();
}
