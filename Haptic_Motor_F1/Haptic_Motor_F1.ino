#include <SimpleFOC.h>
#include <SPI.h>

/* DEFINE LINE ----------------------------------------- */
#define FOC_PID_P 4
#define FOC_PID_I 0
#define FOC_PID_D 0.04
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10

#define FOC_VOLTAGE_POWER_SUPPLY  15
#define FOC_VOLTAGE_LIMIT 5
#define INVERT_ROTATION   1

#define DEAD_ZONE_REG   0.3f
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
const float detent_hysteresis = radians(3);  // Vùng đệm để tránh rung (3°)
const float dead_zone = radians(1);          // vùng chết quanh tâm để tránh rung
const float kp = 6.0f;                       // độ cứng “lò xo” (tăng nếu muốn lực mạnh hơn)
const float kd = 0.05f;                      // giảm chấn theo tốc độ
const float click_strength = 200.0f;           // biên độ xung “click”
const uint8_t click_ms = 2;                  // thời gian mỗi nửa xung (ms)
const float max_no_torque_velocity = 80.0f;  // ngưỡng tốc độ để ngắt torque

// Trạng thái detent
long current_detent_index = 0;
float current_detent_center = 0.0f;
float delta_detent = 0.0f;
float detent_threshold = 0.0f;

// Phát “click” ngắn: xung mô-men + rồi -
void playClick(float strength, uint8_t ms_per_half, float velocity) {
  int check = 1;
  if(velocity > 0) {check = 1;} else { check = -1;}
  motor.move(check * strength);
  for (uint8_t i = 0; i < ms_per_half; i++) { motor.loopFOC(); delay(1); }
  motor.move(-check * strength);
  for (uint8_t i = 0; i < ms_per_half; i++) { motor.loopFOC(); delay(1); }
  motor.move(0);
  motor.loopFOC();
}

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
  sensor.update();
  float a = sensor.getAngle();
  current_detent_index = lround(a / detent_step_rad);
  current_detent_center = current_detent_index * detent_step_rad;

  Serial.println("Ready. Rotate the knob through detents.");
  delay(10);

  motor.sensor_direction = Direction::CCW;
  motor.initFOC();

  motor.monitor_downsample = 0;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
}

bool deadzone_check = true;
void loop() {
  motor.loopFOC();

  // Đo góc hiện tại
  float angle = motor.shaft_angle;
  float vec = motor.shaft_velocity;

  if(deadzone_check){
    // Xác định detent gần nhất (snapping logic)
    long nearest_idx = lround(angle / detent_step_rad);
    // delta_detent = angle & detent_step_rad;
    // Serial.println(nearest_idx);
    if (nearest_idx != current_detent_index) {
      detent_threshold = (nearest_idx * detent_step_rad + current_detent_index * detent_step_rad)/2;
      current_detent_index = nearest_idx;
      current_detent_center = current_detent_index * detent_step_rad;
      // Serial.println(detent_threshold * RAD_TO_DEG);
      deadzone_check = false;

      // Phát “click” khi vào detent mới
      playClick(click_strength, click_ms, vec);

      Serial.print("Detent #"); Serial.print(current_detent_index);
      Serial.print("  angle: "); Serial.print(angle, 3);
      Serial.print(" rad ("); Serial.print(angle * RAD_TO_DEG, 1);
      Serial.println(" deg)");
    }
  }else{
    double delta = fabs(angle - detent_threshold) ;
    Serial.println(delta * RAD_TO_DEG);
    if(delta * RAD_TO_DEG > DEAD_ZONE_REG)
    {
      deadzone_check = true;
    }
  }


  motor.move(0);

  // In trạng thái mỗi 50 ms (tùy chọn)
  static uint32_t last_print = 0;
  if (millis() - last_print > 50) {
    // Serial.print("velocity: "); Serial.print(motor.shaft_velocity, 3);
    Serial.print("angle: "); Serial.print(angle * RAD_TO_DEG, 3);
    // Serial.print("  center: "); Serial.print(current_detent_center, 3);
    // Serial.print("  idx: "); Serial.print(current_detent_index);
    Serial.print("  torque: "); Serial.println(torque, 2);
    last_print = millis();
  }

  delay(1);
}


