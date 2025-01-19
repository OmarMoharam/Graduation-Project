#include "sensors_and_actuators.h"

void setup() {
  setupSensorsAndActuators(); // Initialize sensors and actuators
}

void loop() {
  unsigned long currentMillis = millis(); // Get current time
  if (currentMillis - loopPreviousMillis >= loopInterval) { // Check if loop interval has elapsed
    loopPreviousMillis = currentMillis; // Update loop timestamp

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
    if (millis() - currentSensorTimestamp >= 1000) { // Check if 1 second has elapsed
      currentSensorTimestamp = millis(); // Update current sensor timestamp
      int currentReading = current_sensor.mA_DC(10); // Read current
      Serial.print("Current (mA): "); Serial.println(currentReading);
  
      if (currentReading > SAFEST_CURRENT) { // Check if current exceeds safe limit
        Serial.println("Current exceeds safe limit! Stopping motors.");
        setThrustersNeutral(); // Stop motors
      }
    }
  
    // Pressure and Depth Readings
    clockSetup(clock); // Initialize pressure sensor clock
    if (millis() - pressureSensorTimestamp >= 1000) { // Check if 1 second has elapsed
      pressureSensorTimestamp = millis(); // Update pressure sensor timestamp
      Serial.print("Pressure (mbar): "); Serial.println(getPressureinMBAR());
      Serial.print("Depth (m): "); Serial.println(getDepthinMeter(getPressureinMBAR()));
      Serial.print("Temperature (C): "); Serial.println(getTemperatureinC());
      if (getDepthinMeter(getPressureinMBAR()) > maxDepth) { // Check if ROV depth exceeds safe limit
        Serial.println("Warning! YOU REACHED THE MAXIMUM DEPTH, PLEASE HEAVE UP");
        setThrustersNeutral(); // Stop motors
      }
    }
  
    // Temperature Sensor Monitoring
    if (millis() - temperatureSensorTimestamp >= 1000) { // Check if 1 second has elapsed
      temperatureSensorTimestamp = millis(); // Update temperature sensor timestamp
      Serial.print("Temperature: ");
      tempSensor.Temp_voidPrintTemperature(); // Print temperature
    }
  
    // Lighting Control
    controlLighting(); // Control lighting system

    // Update IMU Data
    updateIMUData(); // Update IMU data and compute PID outputs

    // SportMode control
    controlSportMode(); // Control the SportMode for fast linear speed
  }
}