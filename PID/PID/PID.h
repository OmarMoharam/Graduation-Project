#ifndef PID_H
#define PID_H

#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>

//------------------------------------- PID Controller and MPU Variables ------------------------//

class MPU6050_PID_Controller {
  public:
    MPU6050_PID_Controller(int escPin);

    void setup();
    void update();

  private:
    Adafruit_MPU6050 mpu;
    Servo escPin; // ESC connected to this pin

    double setpoint;
    double input;
    double output;

    double kp;
    double ki;
    double kd;

    PID myPID;

    // Gyroscope and Accelerometer Data
    float gyroRateX;
    float gyroRateY;
    float gyroRateZ;

    float angleAccelX;
    float angleAccelY;
    float angleAccelZ;

    float angleFilteredX;
    float angleFilteredY;
    float angleFilteredZ;

    float dt; // Time step

    void calculateAngles();
    void filterAngles();
    void sendToESC();
    void printData();
};

#endif
