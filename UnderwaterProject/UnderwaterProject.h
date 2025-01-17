#ifndef UNDERWATERPROJECT_H
#define UNDERWATERPROJECT_H

#include <Wire.h>
#include "ACS712.h"
#include "PressureAndDepthSensor.h"
#include "TemperatureSensor.h"
#include <Servo.h>
#include "MPU6050_Sensor.h"
#include "HMC5883L_Sensor.h"
#include <PID_v1.h>

// MPU6050 + Magnetometer Setup
extern MPU6050_Sensor mpu;
extern HMC5883L_Sensor hmc;
extern float rollReading, pitchReading, yawReading;
extern float alpha;

// Current Sensor Setup
#define CURRENT_PIN A0
#define INPUT_VOLTAGE 5.0
#define ARDUINO_ADC 1023
#define SENSOR_SCALE_FACTOR 66
#define SAFEST_CURRENT 8000 // In mA
extern ACS712 current_sensor;
extern unsigned long current_u32PreviousMillis;

// Pressure and Depth Sensor Setup
extern unsigned long pressure_u32PreviousMillis;
extern const int clock;

// Temperature Sensor Setup
#define TEMPERATURE_BUS 2
#define TEMPERATURE_RESOLUTION 9
extern TemperatureSensor tempSensor;
extern unsigned long temperature_u32PreviousMillis;

// Thruster Setup
extern Servo Thruster1, Thruster2, Thruster3, Thruster4, Thruster5, Thruster6;
extern int T1, T2, T3, T4, T5, T6;

// Joystick Variables
extern int groundStationAddress;
extern int surge, sway, yaw, heave, pitch, roll, Button;
extern int cameraValueX, cameraValueY;

// Camera Servo Setup
extern Servo servoX, servoY;
#define SERVO_X_PIN 6
#define SERVO_Y_PIN 7
extern int angleX, angleY;

// Lighting System Setup
#define LIGHTS_PIN 14

// Loop timing
extern unsigned long loopPreviousMillis;
extern const unsigned long loopInterval;

// PID Controller Setup
 do
 double inuble setpoint;put;
 double output;

 double kp;
 double ki;
 double kd;


// Function Declarations
void setupSensorsAndActuators();
void pollJoystick();
void calculateThrusters();
void setThrustersNeutral();
void controlLighting();
void updateIMUData();

#endif
