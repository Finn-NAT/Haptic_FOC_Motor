#ifndef MOTOR_CONTROL_HAPTIC_HPP
#define MOTOR_CONTROL_HAPTIC_HPP

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

extern BLDCMotor motor;
extern BLDCDriver3PWM driver;
extern MagneticSensorSPI sensor;
extern SPIClass spiBus;

extern float max_position;
extern float min_position;

/* USER DEFINE LINE ------------------------------------ */
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23
#define PIN_SPI_CS    5

#define DRIVER_VOLTAGE_POWER_SUPPLY  20

#define FOC_VOLTAGE_LIMIT 15
#define FOC_PID_P_DEFAULT 700
#define FOC_PID_I_DEFAULT 0.005
#define FOC_PID_D_DEFAULT 0.004
#define FOC_PID_PV_DEFAULT 0.5
#define FOC_PID_IV_DEFAULT 0.1
#define FOC_LOW_PASS_FILTER_VELOCITY 0.04
#define FOC_PID_VELOCITY_LIMIT 20
#define FOC_VOLTAGE_SENSOR_ALIGN 16
#define FOC_SENSOR_DIRCTION  Direction::CCW
#define FOC_ZERO_ELECTRIC_ANGLE  2.783409f

#define CALIB_PD_P_VALUE 140.0f
#define CALIB_PD_D_VALUE 0.1f
#define CALIB_TORQUE_VALUE 7.0f

#define HAPTIC_OUT_ANGLE_DEFAULT (7.0f * PI / 180.0)
#define HAPTIC_IN_ANGLE_DEFAULT  (1.0f * PI / 180.0)
/* ----------------------------------------------------- */

typedef struct haptic_point {
    float haptic_detent_angle;
    float haptic_zone_angle;
    float haptic_out_angle;
    float haptic_in_angle;
} haptic_point_t;

void MotorHaptic_Init();
void MotorHaptic_Loop();
void MotorHaptic_Calib();

float MotorDriver_SetTorque_P(float percent);
float MotorDriver_SetTorque_V(float value);

//float MotorDriver_SetAngle_V(float value);
//float MotorDriver_SetVelocity_V(float value);

float MotorDriver_GetAngle();
float MotorDriver_GetVelocity();


#endif 
