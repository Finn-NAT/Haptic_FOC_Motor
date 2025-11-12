#include <SimpleFOC.h>
#include <SPI.h>

/* DEFINE LINE ----------------------------------------- */
#define FOC_PID_P 4
#define FOC_PID_I 0.005
#define FOC_PID_D 0.1
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10

#define FOC_VOLTAGE_POWER_SUPPLY  15
#define FOC_VOLTAGE_LIMIT 5
#define INVERT_ROTATION   1
/* ----------------------------------------------------- */

typedef struct Haptic_Config {
    int32_t position;
    float sub_position_unit;
    uint8_t position_nonce;
    int32_t min_position;
    int32_t max_position;
    float position_width_radians;
    float detent_strength_unit;
    float endstop_strength_unit;
    float snap_point;
    char text[51];
    int detent_positions_count;
    int32_t detent_positions[5];
    float snap_point_bias;
    int16_t led_hue;
} Haptic_Config;

Haptic_Config config = {
    .position = 0,
    .sub_position_unit = 0,
    .position_nonce = 7,
    .min_position = 0,
    .max_position = 31,
    .position_width_radians = 7 * _PI / 180,
    .detent_strength_unit = 2.5,
    .endstop_strength_unit = 1,
    .snap_point = 0.7,
    .detent_positions_count = 4,
    .detent_positions = {2, 10, 21, 22},
    .snap_point_bias = 0,
};

// Haptic_Config config = {
//     .position = 0,
//     .sub_position_unit = 0,
//     .position_nonce = 3,
//     .min_position = 0,
//     .max_position = 1,
//     .position_width_radians = 60 * _PI / 180,
//     .detent_strength_unit = 1,
//     .endstop_strength_unit = 1,
//     .snap_point = 0.55,
//     .detent_positions_count = 0,
//     .snap_point_bias = 0,
// };

// Haptic_Config config = {
//     .position = 3,
//     .sub_position_unit = 0,
//     .position_nonce = 5,
//     .min_position = 0,
//     .max_position = 6,
//     .position_width_radians = 30 * _PI / 180,
//     .detent_strength_unit = 1,
//     .endstop_strength_unit = 1,
//     .snap_point = 1.1,
//     .detent_positions_count = 0,
//     .snap_point_bias = 0,
// };

float current_detent_center;
int32_t current_position;
float latest_sub_position_unit;
float idle_check_velocity_ewma;
uint32_t last_idle_start;
uint32_t last_publish;

const float DEAD_ZONE_DETENT_PERCENT = 0.2;
const float DEAD_ZONE_RAD = 1 * _PI / 180;

static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

BLDCMotor motor = BLDCMotor(7);         
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

float CLAMP(const float value, const float low, const float high) 
{
  return value < low ? low : (value > high ? high : value); 
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

  sensor.update();
  delay(10);

  motor.sensor_direction = Direction::CCW;
  motor.initFOC();

  motor.monitor_downsample = 0;

  current_detent_center = motor.shaft_angle;
  current_position = 0;
  latest_sub_position_unit = 0;

  idle_check_velocity_ewma = 0;
  last_idle_start = 0;
  last_publish = 0;
  Serial.begin(115200);
  motor.useMonitoring(Serial);
}

void loop() {
  motor.loopFOC();

  float angle_to_detent_center = motor.shaft_angle - current_detent_center;
#if INVERT_ROTATION
  angle_to_detent_center = -motor.shaft_angle - current_detent_center;
#endif

  float snap_point_radians = config.position_width_radians * config.snap_point;
  float bias_radians = config.position_width_radians * config.snap_point_bias;
  float snap_point_radians_decrease = snap_point_radians + (current_position <= 0 ? bias_radians : -bias_radians);
  float snap_point_radians_increase = -snap_point_radians + (current_position >= 0 ? -bias_radians : bias_radians); 

  int32_t num_positions = config.max_position - config.min_position + 1;
  if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || current_position > config.min_position)) {
    current_detent_center += config.position_width_radians;
    angle_to_detent_center -= config.position_width_radians;
    current_position--;
  } else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || current_position < config.max_position)) {
    current_detent_center -= config.position_width_radians;
    angle_to_detent_center += config.position_width_radians;
    current_position++;
  }

  latest_sub_position_unit = -angle_to_detent_center / config.position_width_radians;

  float dead_zone_adjustment = CLAMP(
    angle_to_detent_center,
    fmaxf(-config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
    fminf(config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

  bool out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && current_position == config.min_position) || (angle_to_detent_center < 0 && current_position == config.max_position));
  motor.PID_velocity.limit = 10; //out_of_bounds ? 10 : 3;
  motor.PID_velocity.P = out_of_bounds ? config.endstop_strength_unit * 4 : config.detent_strength_unit * 4;

  // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
  if (fabsf(motor.shaft_velocity) > 60) {
    // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
    motor.move(0);
  } else {
    float input = -angle_to_detent_center + dead_zone_adjustment;
    if (!out_of_bounds && config.detent_positions_count > 0) {
      bool in_detent = false;
      for (uint8_t i = 0; i < config.detent_positions_count; i++) {
        if (config.detent_positions[i] == current_position) {
          in_detent = true;
          break;
        }
      }
      if (!in_detent) {
        input = 0;
      }
    }
    float torque = motor.PID_velocity(input);
#if INVERT_ROTATION
    torque = -torque;
#endif
    motor.move(torque);
  }
  // Serial.println(torque);

  delay(1);

}


