#include <SimpleFOC.h>
#include <SPI.h>

/* DEFINE LINE ----------------------------------------- */
#define FOC_PID_P 50
#define FOC_PID_I 0
#define FOC_PID_D 0.01
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10

#define FOC_VOLTAGE_POWER_SUPPLY  15
#define FOC_VOLTAGE_LIMIT 8
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
float breakaway_angle_rad = radians(0.3); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
const float detent_hysteresis = radians(3); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
const float dead_angle_rad = radians(1.5); // Ngưỡng góc để "bẻ gãy" lực cản (2.5°)
float kp = 8000.0f;                      // Độ cứng của "bức tường" cản
float kd = 0.01f;                       // Giảm chấn để ổn định
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


void setup() {

  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
  sensor.init(&spiBus);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = FOC_VOLTAGE_POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;

  motor.velocity_limit = 10000;
  motor.PID_velocity.P = FOC_PID_P;
  motor.PID_velocity.I = FOC_PID_I;
  motor.PID_velocity.D = FOC_PID_D;
  motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP;
  motor.PID_velocity.limit = FOC_PID_LIMIT;

  motor.init();

  // Căn tâm detent ban đầu tại vị trí gần nhất
  // sensor.update();
  // float a = sensor.getAngle();
  // current_detent_index = lround(a / detent_step_rad);
  // current_detent_center = a;

  Serial.println("Ready. Rotate the knob through detents.");
  delay(10);

  motor.sensor_direction = Direction::CCW;
  motor.initFOC();

  motor.monitor_downsample = 0;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor.loopFOC();
  current_detent_center = motor.shaft_angle;
  command.add('P', doTargetP, "target P");
  command.add('D', doTargetD, "target D");
}

bool deadzone_check = true;
float idle_check_velocity_ewma = 0;
    uint32_t last_idle_start = 0;
    uint32_t last_publish = 0;
void loop() {
  motor.loopFOC();
  command.run();

        // idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
        // if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
        //     last_idle_start = 0;
        // } else {
        //     if (last_idle_start == 0) {
        //         last_idle_start = millis();
        //     }
        // }
        // if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
        //     current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
        //     Serial.println(idle_check_velocity_ewma);Serial.print(" -------------------------------------- ");Serial.println(current_detent_center * RAD_TO_DEG);
        // }

  // Đo góc hiện tại
  float angle = motor.shaft_angle;
  // float vec = motor.shaft_velocity;

  // Tính toán lực cản "bức tường"
  float error =  angle - current_detent_center;
  float abs_error =  fabs(error);
  //Serial.println(error);
  if(deadzone_check)
  {
    torque_T = - kp * error * error * (error > 0 ? 1 : -1) - kd * motor.shaft_velocity ;
    Serial.print(torque_T);Serial.print(" - ");Serial.println(error * RAD_TO_DEG);
    if(torque_T < 0.55 && torque_T > -0.55) {torque_T = 0;}
  }

  // if(abs_error <= breakaway_angle_rad){
  //   torque_T = 0;
  // }else{
  //   deadzone_check = true;
  // }

  

  if( fabs(error) > dead_angle_rad ){
    deadzone_check = false;
    torque_T = 0;
  }else{
    deadzone_check = true;
  }

  // }
  // // Nếu vượt ngưỡng, không áp dụng lực cản, cho phép quay tự do
  // else {
  //   torque_T = 0;
  // }
  motor.move(torque_T);
  

  // In trạng thái mỗi 50 ms (tùy chọn)
  // static uint32_t last_print = 0;
  // if (millis() - last_print > 50) {
  //   // Serial.print("velocity: "); Serial.print(motor.shaft_velocity, 3);
  //   Serial.print("angle: "); Serial.print(angle * RAD_TO_DEG, 3);
  //   // Serial.print("  center: "); Serial.print(current_detent_center, 3);
  //   // Serial.print("  idx: "); Serial.print(current_detent_index);
  //   Serial.print("  torque: "); Serial.println(torque, 2);
  //   last_print = millis();
  // }

  delay(1);
}


