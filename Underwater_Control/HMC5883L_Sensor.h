#ifndef HMC5883L_SENSOR_H
#define HMC5883L_SENSOR_H

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

class HMC5883L_Sensor {
public:
  HMC5883L_Sensor();
  void begin();
  void calibrate();
  void update();
  float getYaw();

private:
  Adafruit_HMC5883_Unified mag;
  float yaw;
  float offsetX, offsetY, offsetZ;
  float scaleX, scaleY, scaleZ;
};

#endif