#include <Wire.h>
#include <MPU6050_tockn.h>
#include "ACS712.h"
#include "PressureAndDepthSensor.h"
#include "TemperatureSensor.h"
#include <Servo.h>

// MPU6050 Setup
MPU6050 mpu6050(Wire);
long MPU_longTimer = 0;

// Current Sensor Setup
#define CURRENT_PIN A0
#define INPUT_VOLTAGE 5.0
#define ARDUINO_ADC 1023
#define SENSOR_SCALE_FACTOR 66
#define SAFEST_CURRENT 8000 // In mA
ACS712 current_sensor(CURRENT_PIN, INPUT_VOLTAGE, ARDUINO_ADC, SENSOR_SCALE_FACTOR);
unsigned long current_u32PreviousMillis = 0;

// Pressure and Depth Sensor Setup
unsigned long pressure_u32PreviousMillis = 0;
const int clock = 8;

// Temperature Sensor Setup
#define TEMPERATURE_BUS 2
#define TEMPERATURE_RESOLUTION 9
TemperatureSensor tempSensor(TEMPERATURE_BUS, TEMPERATURE_RESOLUTION);
unsigned long temperature_u32PreviousMillis = 0;

// Thruster Setup
Servo Thruster1; // Forward-right (45°)
Servo Thruster2; // Forward-left (135°)
Servo Thruster3; // Backward-right (315°)
Servo Thruster4; // Backward-left (225°)
Servo Thruster5; // Vertical-right
Servo Thruster6; // Vertical-left

int T1 = 0, T2 = 0, T3 = 0, T4 = 0, T5 = 0, T6 = 0;

// Joystick Variables
int groundStationAddress = 8;
int surge = 0, sway = 0, yaw = 0, heave = 0, pitch = 0, roll = 0, Button = 0;
int cameraValueX = 0, cameraValueY = 0;

// Camera Servo Setup
Servo servoX, servoY; // Camera servos
#define SERVO_X_PIN 6
#define SERVO_Y_PIN 7
int angleX = 90, angleY = 90;

// Lighting System Setup
#define LIGHTS_PIN 14

// Loop delay time
unsigned long loopPreviousMillis = 0;
const unsigned long loopInterval = 50; // 50 ms

void setup() {
  Serial.begin(115200);
  
  // MPU6050 Initialization
  Wire.begin();
  mpu6050.begin();

  /*  in this section we should measure the offset in normal postion and
      then put the numbers in the setGyroOffsets
  mpu6050.calcGyroOffsets(true);
  delay(50000); */
  mpu6050.setGyroOffsets(-5.49, 0.64, -0.75);

  // Current Sensor Initialization
  current_sensor.autoMidPoint(100);
  Serial.print("Current Sensor Midpoint: ");
  Serial.println(current_sensor.getMidPoint());

  // Pressure Sensor Initialization
  SPISetup(clock);

  // Temperature Sensor Initialization
  tempSensor.Temp_voidBegin();
  Serial.print("Parasite power is: ");
  Serial.println(tempSensor.Temp_boolIsParasitePowered() ? "ON" : "OFF");
  
  // Thruster Initialization
  Thruster1.attach(9);
  Thruster2.attach(10);
  Thruster3.attach(11);
  Thruster4.attach(12);
  Thruster5.attach(13);
  Thruster6.attach(14);

  // Neutral positions
  setThrustersNeutral();

  // Camera Servo Initialization
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(angleX);
  servoY.write(angleY);
  Serial.println("Camera Servo Control Initialized");

  // Lighting Initialization
  pinMode(LIGHTS_PIN, OUTPUT);
  digitalWrite(LIGHTS_PIN, LOW); // Lights off by default
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - loopPreviousMillis >= loopInterval) {
    loopPreviousMillis = currentMillis;

    pollJoystick(); // Poll joystick data

    // Thruster calculations and outputs
    calculateThrusters();
  
    // Apply thruster outputs
    Thruster1.writeMicroseconds(constrain(map(T1, -100, 100, 1100, 1900), 1100, 1900));
    Thruster2.writeMicroseconds(constrain(map(T2, -100, 100, 1100, 1900), 1100, 1900));
    Thruster3.writeMicroseconds(constrain(map(T3, -100, 100, 1100, 1900), 1100, 1900));
    Thruster4.writeMicroseconds(constrain(map(T4, -100, 100, 1100, 1900), 1100, 1900));
    Thruster5.writeMicroseconds(constrain(map(T5, -100, 100, 1100, 1900), 1100, 1900));
    Thruster6.writeMicroseconds(constrain(map(T6, -100, 100, 1100, 1900), 1100, 1900));
  
    delay(50); // Refresh rate
  
    // MPU6050 Readings
    mpu6050.update();
    if (millis() - MPU_longTimer > 1000) {
      Serial.print("AngleX: "); Serial.print(mpu6050.getAngleX());
      Serial.print("\tAngleY: "); Serial.print(mpu6050.getAngleY());
      Serial.print("\tAngleZ: "); Serial.println(mpu6050.getAngleZ());
      MPU_longTimer = millis();
    }
  
    // Current Sensor Monitoring
    if (millis() - current_u32PreviousMillis >= 1000) {
      current_u32PreviousMillis = millis();
      int currentReading = current_sensor.mA_DC(10);
      Serial.print("Current (mA): "); Serial.println(currentReading);
  
      if (currentReading > SAFEST_CURRENT) {
        Serial.println("Current exceeds safe limit! Stopping motors.");
        setThrustersNeutral();
      }
    }
  
    // Pressure and Depth Readings
    clockSetup(clock);
    if (millis() - pressure_u32PreviousMillis >= 1000) {
      pressure_u32PreviousMillis = millis();
      Serial.print("Pressure (mbar): "); Serial.println(getPressureinMBAR());
      Serial.print("Depth (m): "); Serial.println(getDepthinMeter(getPressureinMBAR()));
      Serial.print("Temperature (C): "); Serial.println(getTemperatureinC());
    }
  
    // Temperature Sensor Monitoring
    if (millis() - temperature_u32PreviousMillis >= 1000) {
      temperature_u32PreviousMillis = millis();
      Serial.print("Temperature: ");
      tempSensor.Temp_voidPrintTemperature();
    }
  
    // Camera Control
    angleX = map(cameraValueX, 0, 1023, 0, 180);
    angleY = map(cameraValueY, 0, 1023, 0, 180);
    servoX.write(angleX);
    servoY.write(angleY);
  
    // Lighting Control
    controlLighting();
    }
}
  
}

void setThrustersNeutral() {
  Thruster1.writeMicroseconds(1500);
  Thruster2.writeMicroseconds(1500);
  Thruster3.writeMicroseconds(1500);
  Thruster4.writeMicroseconds(1500);
  Thruster5.writeMicroseconds(1500);
  Thruster6.writeMicroseconds(1500);
}


void pollJoystick() {
  Wire.beginTransmission(groundStationAddress);
  Wire.write(1); // Request joystick data
  Wire.endTransmission();

  Wire.requestFrom(groundStationAddress, 9);
  if (Wire.available() == 9) {
    surge = Wire.read();
    sway = Wire.read();
    yaw = Wire.read();
    heave = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
    Button = Wire.read();
    cameraValueX = Wire.read();
    cameraValueY = Wire.read();
  } else {
    Serial.println("Error: Incomplete joystick data received.");
    // Optionally, set all joystick values to neutral (0)
    surge = sway = yaw = heave = pitch = roll = Button = cameraValueX = cameraValueY = 0;
  }
}

void calculateThrusters() {
  T1 = surge + sway + yaw;
  T2 = surge - sway - yaw;
  T3 = -surge + sway + yaw;
  T4 = -surge - sway - yaw;
  T5 = heave + pitch;
  T6 = heave - pitch;
}

void controlLighting() {
  if (Button == 1) {
    digitalWrite(LIGHTS_PIN, HIGH);
    Serial.println("Light ON");
  } else {
    digitalWrite(LIGHTS_PIN, LOW);
    Serial.println("Light OFF");
  }
}