#include "MPU6050_Sensor.h"
#include "HMC5883L_Sensor.h"

MPU6050_Sensor mpu;
HMC5883L_Sensor hmc;

// Complementary Filter variables
float roll = 0, pitch = 0, yaw = 0;
float alpha = 0.98; // Complementary filter coefficient

void setup() {
  Serial.begin(9600);

  // Initialize sensors
  mpu.begin();
  hmc.begin();

  // Calibrate sensors (if needed)
  //mpu.calibrate();
  //hmc.calibrate();
}

void loop() {
  // Update sensor readings
  mpu.update();
  hmc.update();

  // Get raw sensor data
  float rawRoll = mpu.getRoll();
  float rawPitch = mpu.getPitch();
  float rawYaw = hmc.getYaw();

  // Apply Complementary Filter
  roll = alpha * roll + (1 - alpha) * rawRoll;
  pitch = alpha * pitch + (1 - alpha) * rawPitch;
  yaw = alpha * yaw + (1 - alpha) * rawYaw;

  // Print final roll, pitch, and yaw
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.println(yaw);

  delay(100); // Adjust delay for faster/slower sampling
}