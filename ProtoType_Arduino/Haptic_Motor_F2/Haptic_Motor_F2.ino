#include <SimpleFOC.h>
#include <SPI.h>

/* DEFINE LINE ----------------------------------------- */
#define PID_P 140
#define PID_I 0.1
#define PID_D 0.1
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 15

#define FOC_VOLTAGE_POWER_SUPPLY  20
#define FOC_VOLTAGE_LIMIT 18
#define INVERT_ROTATION   1

#define DEAD_ZONE_REG   0.25f
/* ----------------------------------------------------- */

BLDCMotor motor = BLDCMotor(7);         
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

// ===== Tham số haptic/detent =====
const float detent_step_rad = radians(10);   // khoảng cách giữa các vị trí cố định (10°)
float breakaway_angle_rad = radians(0.15); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
const float detent_hysteresis = radians(3); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
const float dead_angle_rad = radians(1.5); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
// float kp = 8000.0f;                      // Độ cứng của "bức tường" cản
// float kd = 0.01f;                       // Giảm chấn để ổn định
// float kp = 10940.0f;                      // Độ cứng của "bức tường" cản
// float kd = 0.32f;                       // Giảm chấn để ổn định
float kp = 17600.0f;                      // Độ cứng của "bức tường" cản
float kd = 1.0f;                       // Giảm chấn để ổn định
float ki = 0.05f;                       // Giảm chấn để ổn định
const float click_strength = 200.0f;           // biên độ xung “click”
const uint8_t click_ms = 1;                  // thời gian mỗi nửa xung (ms)

// Trạng thái detent
long current_detent_index = 0;
float current_detent_center = 0.0f;
float delta_detent = 0.0f;
float detent_threshold = 0.0f;

float torque_T = 0;

static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
static const float DEAD_ZONE_RAD = 1 * _PI / 180;

static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0001;


Commander command = Commander(Serial);
void doTargetP(char* cmd) { command.scalar(&kp, cmd); }
void doTargetD(char* cmd) { command.scalar(&kd, cmd); }

bool one_time = true;
float min_position = 0;
float max_position = 0;
void setup() {

  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.init(&spiBus);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = FOC_VOLTAGE_POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;

  motor.LPF_velocity.Tf = 0.03;
  // motor.PID_velocity.P = FOC_PID_P;
  // motor.PID_velocity.I = FOC_PID_I;
  // motor.PID_velocity.D = FOC_PID_D;
  motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP;
  motor.PID_velocity.limit = FOC_PID_LIMIT;

  motor.init();

  // Căn tâm detent ban đầu tại vị trí gần nhất
  // sensor.update();
  // float a = sensor.getAngle();
  // current_detent_index = lround(a / detent_step_rad);
  // current_detent_center = a;

  // Serial.println("Ready. Rotate the knob through detents.");
  // delay(10);
  motor.voltage_sensor_align = 16;
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 2.783409;
  motor.initFOC();
  Serial.println(motor.zero_electric_angle, 6);

  motor.monitor_downsample = 0;

  Serial.begin(115200);
  Serial.println(motor.zero_electric_angle, 6);
  motor.useMonitoring(Serial);
  motor.loopFOC();
  float old_angle = motor.shaft_angle;
  int countt = 0;
  while(1){
    for(int i = 0 ; i < 40; i++){
      motor.loopFOC();
      motor.move(7);
      delay(1);
    }
    //Serial.println((motor.shaft_angle - old_angle)*RAD_TO_DEG);
    if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
      countt++;
      if(countt > 20){
        max_position = motor.shaft_angle;
        //Serial.println(motor.shaft_angle * RAD_TO_DEG);
        break;
      }

    }
    old_angle = motor.shaft_angle; 
  }
  countt = 0;
  while(1){
    for(int i = 0 ; i < 40; i++){
      motor.loopFOC();
      motor.move(-7);
      delay(1);
    }
    //Serial.println((motor.shaft_angle - old_angle)*RAD_TO_DEG);
    if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
      countt++;
      if(countt > 20){
        min_position = motor.shaft_angle;
        //Serial.println(motor.shaft_angle * RAD_TO_DEG);
        break;
      }
    }
    old_angle = motor.shaft_angle; 
  }
  command.add('P', doTargetP, "target P");
  command.add('D', doTargetD, "target D");
  current_detent_center = (max_position + min_position)/2;
  int count = 0;
  while(1){
    motor.loopFOC();
    float angle = motor.shaft_angle;
    float vec = motor.shaft_velocity;
    float error =  angle - current_detent_center;
    torque_T = - (error * PID_P + PID_D * vec);
    if (fabs(torque_T) > 6) {
      torque_T = torque_T > 0 ? 6 : -6;
    }
    motor.move(torque_T);
    if(fabs(error*RAD_TO_DEG) < 1.5){count++;}
    if(count > 50) {break;}
    Serial.print(torque_T);Serial.print(" - ");Serial.println(error*RAD_TO_DEG);
    delay(1);
  }

  torque_T = 0;
}

bool deadzone_check = true;
float previous_vec = 0;

float pre_error = 0;
float total_error = 0;  
void loop() {
  motor.loopFOC();
  command.run();

  // Đo góc hiện tại
  float angle = motor.shaft_angle;
  // float vec = motor.shaft_velocity;

  // Tính toán lực cản "bức tường"
  float error =  angle - current_detent_center;
  float abs_error =  fabs(error);
  //Serial.println(angle * RAD_TO_DEG);
  if(deadzone_check)
  {
    //float i = ki*total_error;
    torque_T = - kp * error * error * (error > 0 ? 1 : -1) - kd * motor.shaft_velocity + ki*total_error;
    Serial.print(torque_T);Serial.print(" - ");Serial.print(ki*total_error);Serial.print(" - ");Serial.println(error * RAD_TO_DEG);
    if(torque_T < 2 && torque_T > -2) {total_error += torque_T;}
    if(fabs(error) < breakaway_angle_rad){total_error = 0; torque_T = 0;}
  }

  if( fabs(error) > dead_angle_rad ){
    deadzone_check = false;
    // torque_T =  - 1 * motor.shaft_velocity;
    torque_T =  0;
    total_error = 0;
  }else{
    deadzone_check = true;
  }

  motor.move(torque_T);

  delayMicroseconds(500);
}


