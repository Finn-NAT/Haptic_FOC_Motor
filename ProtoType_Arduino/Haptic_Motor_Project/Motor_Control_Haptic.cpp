#include "Motor_Control_Haptic.h"

// Definitions of haptic motor, driver and sensor---------------------------//
BLDCMotor motor = BLDCMotor(7);

BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

SPIClass spiBus(VSPI);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);
// ------------------------------------------------------------------------//

float max_position = 0;
float min_position = 0;
float center_position = 0;

haptic_point_t zero_point = {0.0f, HAPTIC_OUT_ANGLE_DEFAULT, HAPTIC_OUT_ANGLE_DEFAULT, HAPTIC_IN_ANGLE_DEFAULT};

void MotorHaptic_Init() {
	// Init SENSOR ------------------------------------------------------//
	spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
	sensor.clock_speed = 50000;
	sensor.init(&spiBus);
	motor.linkSensor(&sensor);
	// ------------------------------------------------------------------//

	// Init DRIVER ------------------------------------------------------//
	driver.voltage_power_supply = DRIVER_VOLTAGE_POWER_SUPPLY;
	driver.init();
	motor.linkDriver(&driver);
	// ------------------------------------------------------------------//

	// Haptic Motor settings (Using FOC)---------------------------------//
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor.controller = MotionControlType::torque;
  	//Speed PID Setting                                     
  	motor.PID_velocity.P = FOC_PID_PV_DEFAULT;            
  	motor.PID_velocity.I = FOC_PID_IV_DEFAULT;
  	//Angle PID Setting 
  	motor.P_angle.P = FOC_PID_P_DEFAULT;
  	motor.P_angle.D = FOC_PID_D_DEFAULT;
  	motor.P_angle.I = FOC_PID_I_DEFAULT;

	motor.voltage_limit = FOC_VOLTAGE_LIMIT;
	motor.LPF_velocity.Tf = FOC_LOW_PASS_FILTER_VELOCITY;
	motor.PID_velocity.limit = FOC_PID_VELOCITY_LIMIT;
	motor.init();
	motor.voltage_sensor_align = FOC_VOLTAGE_SENSOR_ALIGN;
	motor.sensor_direction = FOC_SENSOR_DIRCTION;
	motor.zero_electric_angle = 2.783409;
	motor.initFOC();
	// ------------------------------------------------------------------//
}

void MotorHaptic_Calib() {
	motor.loopFOC();
	float old_angle = motor.shaft_angle;
	int count = 0;
	while(1){
	  for(int i = 0 ; i < 40; i++){
		motor.loopFOC();
		motor.move(CALIB_TORQUE_VALUE);
		delay(1);
	  }
	  if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		count++;
		if(count > 20){
		  max_position = motor.shaft_angle;
		  break;
		}
  
	  }
	  old_angle = motor.shaft_angle; 
	}

	count = 0;
	while(1){
	  for(int i = 0 ; i < 40; i++){
		motor.loopFOC();
		motor.move(-CALIB_TORQUE_VALUE);
		delay(1);
	  }
	  if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		count++;
		if(count > 20){
		  min_position = motor.shaft_angle;
		  break;
		}
	  }
	  old_angle = motor.shaft_angle; 
	}

	center_position = (max_position + min_position)/2;
	count = 0;
	while(1){
	  motor.loopFOC();
	  float angle = motor.shaft_angle;
	  float vel = motor.shaft_velocity;
	  float error =  angle - center_position;
	  float torque_T = - (error * CALIB_PD_P_VALUE + CALIB_PD_D_VALUE * vel);
	  if (fabs(torque_T) > CALIB_TORQUE_VALUE) {
		torque_T = torque_T > 0 ? CALIB_TORQUE_VALUE : -CALIB_TORQUE_VALUE;
	  }
	  motor.move(torque_T);
	  if(fabs(error*RAD_TO_DEG) < 1){count++;}
	  if(count > 50) {break;}
	  delay(1);
	}  
	zero_point.haptic_detent_angle = (min_position + max_position) / 2.0f;
}

void MotorHaptic_Loop() {
	bool one_time_flag = true;
	while(true){
		motor.loopFOC();
		float error = fabs(zero_point.haptic_detent_angle - motor.shaft_angle);
		if (error < zero_point.haptic_zone_angle) {
			zero_point.haptic_zone_angle  = zero_point.haptic_out_angle;
			if(one_time_flag){
			motor.move(15);
			motor.loopFOC();
			delayMicroseconds(500);
			motor.move(12);
			motor.loopFOC();
			delayMicroseconds(500);
			motor.P_angle.reset();
			motor.PID_velocity.reset();
			motor.controller = MotionControlType::angle;
			one_time_flag = false;
			}
			motor.move(zero_point.haptic_detent_angle);
		}else {
			one_time_flag = true;
			zero_point.haptic_zone_angle  =  zero_point.haptic_in_angle;
			motor.controller = MotionControlType::torque;
			motor.move(0);
		}
		delayMicroseconds(1000);
	}
	
}