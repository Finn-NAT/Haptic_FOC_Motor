#include <SimpleFOC.h>
#include <SPI.h>

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);

BLDCMotor motor = BLDCMotor(7, 5.48, NOT_SET, 0.007);         //According to the selected motor, modify the value in BLDCMotor()
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

// Cảm biến AS5048A qua SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // power supply voltage
  driver.voltage_power_supply = 20;
  driver.init();
  motor.linkDriver(&driver);

  // initialise magnetic sensor hardware
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  // sensor.spi_mode = SPI_MODE0;
  sensor.init(&spiBus);
  //Connect the Motor Object with the Sensor Object
  motor.linkSensor(&sensor);

  // aligning voltage 
  motor.voltage_sensor_align = 7;
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // force direction search - because default is CW
  motor.sensor_direction = Direction::UNKNOWN;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  
  Serial.println("Sensor zero offset is:");
  Serial.println(motor.zero_electric_angle, 7);
  Serial.println("Sensor natural direction is: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");

  Serial.println("To use these values add them to the code:");
  Serial.print("   motor.sensor_direction=");
  Serial.print(motor.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");
  Serial.println(";");
  Serial.print("   motor.zero_electric_angle=");
  Serial.print(motor.zero_electric_angle, 7);
  Serial.println(";");

  _delay(1000);
  Serial.println("If motor is not moving the alignment procedure was not successfull!!");
}

void loop() {
    
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  //motor.move(2);
  motor.move(0);
}