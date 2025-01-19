#ifndef SENSORS_AND_ACTUATORS_H
#define SENSORS_AND_ACTUATORS_H

#include <Wire.h>
#include "ACS712.h"
#include "PressureAndDepthSensor.h"
#include "TemperatureSensor.h"
#include <Servo.h>
#include "MPU6050_Sensor.h"
#include "HMC5883L_Sensor.h"
#include <PID_v1_bc.h> // Include PID library
#include <Stepper.h> // Include Stepper Motor

// MPU6050 + Magnetometer Setup
extern MPU6050_Sensor mpu; // MPU6050 sensor for roll, pitch, and yaw measurements
extern HMC5883L_Sensor hmc; // Magnetometer for yaw measurement
extern float rollReading, pitchReading, yawReading; // Filtered roll, pitch, and yaw angles
extern float alpha; // Complementary filter coefficient

// Current Sensor Setup
#define CURRENT_PIN A0 // Pin connected to the current sensor
#define INPUT_VOLTAGE 5.0 // Input voltage for the current sensor
#define ARDUINO_ADC 1023 // Maximum ADC value for the current sensor
#define SENSOR_SCALE_FACTOR 66 // Scale factor for current sensor calibration
#define SAFEST_CURRENT 8000 // Maximum safe current in mA
extern ACS712 current_sensor; // Current sensor object
extern unsigned long currentSensorTimestamp; // Timestamp for current sensor readings

// Pressure and Depth Sensor Setup
extern unsigned long pressureSensorTimestamp; // Timestamp for pressure sensor readings
extern const int clock; // Clock pin for pressure sensor
extern int maxDepth, minDepth; // Maximum and minimum depth limits

// Temperature Sensor Setup
#define TEMPERATURE_BUS 2 // Pin connected to the temperature sensor
#define TEMPERATURE_RESOLUTION 9 // Resolution for temperature sensor
extern TemperatureSensor tempSensor; // Temperature sensor object
extern unsigned long temperatureSensorTimestamp; // Timestamp for temperature sensor readings

// Thruster Setup
extern Servo Thruster1, Thruster2, Thruster3, Thruster4, Thruster5, Thruster6; // Thruster servo objects
extern int T1, T2, T3, T4, T5, T6; // Thruster output values

// Joystick Variables
extern int groundStationAddress; // I2C address of the ground station
extern int surge, sway, yaw, heave, pitch, roll, Button; // Joystick input values

// Lighting System Setup
#define LIGHTS_PIN 14 // Pin connected to the lighting system
extern bool isLightOn; // Lighting system state

// Loop timing
extern unsigned long loopPreviousMillis; // Timestamp for loop timing
extern const unsigned long loopInterval; // Loop interval in milliseconds

// PID Controller Setup
extern double setpointRoll, setpointPitch, setpointYaw; // Desired roll, pitch, and yaw angles
extern double setpointHeave; // Desired depth
extern double inputRoll, inputPitch, inputYaw; // Actual roll, pitch, and yaw angles
extern double inputHeave; // Actual depth
extern double outputRoll, outputPitch, outputYaw; // PID outputs for roll, pitch, and yaw
extern double outputHeave; // PID output for depth
extern double kp, ki, kd; // PID tuning parameters
extern PID rollPID, pitchPID, yawPID; // PID controllers for roll, pitch, and yaw
extern PID heavePID; // PID controller for depth

// Stepper Motor Setup
extern const int stepsPerRevolutoin;
extern const float anglePerStep;
extern const int targetAngle;
extern int stepsToMove;
extern char directionFlag;
extern Stepper sportModeStepper;

// Function Declarations
void setupSensorsAndActuators(); // Initialize sensors and actuators
void pollJoystick(); // Read joystick data
void calculateThrusters(); // Calculate thruster outputs
void setThrustersNeutral(); // Set thrusters to neutral position
void controlLighting(); // Control the lighting system
void updateIMUData(); // Update IMU data and compute PID outputs
void controlSportMode(); // Open Sport Mode in the ROV

#endif