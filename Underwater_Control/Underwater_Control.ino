#include "sensors_and_actuators.h"

void setup() {
  setupSensorsAndActuators();
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

    // Update IMU Data
    updateIMUData();
  }
}