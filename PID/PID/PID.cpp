#include "PID.h"
#include <Servo.h>


class MPU6050_PID_Controller {
private:
    Servo escPin1; 
    Servo escPin2;
    Servo escPin3;
    Servo escPin4;
    Servo escPin5;
    Servo escPin6;

    float setpoint, input, output;
    float kp, ki, kd;
    PID myPID;
    float gyroRateX, gyroRateY, gyroRateZ;
    float angleAccelX, angleAccelY, angleAccelZ;
    float angleFilteredX, angleFilteredY, angleFilteredZ;
    float angleYaw, throttle, vertical;
    float dt;

public:
    MPU6050_PID_Controller(int escPin1, int escPin2, int escPin3, int escPin4, int escPin5, int escPin6);
    void setup();
    void update();
};

MPU6050_PID_Controller::MPU6050_PID_Controller(int escPin1, int escPin2, int escPin3, 
                                               int escPin4, int escPin5, int escPin6)
    : setpoint(0), input(0), output(0), 
      kp(10.0), ki(10.0), kd(1.0), myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT), 
      gyroRateX(0), gyroRateY(0), gyroRateZ(0), 
      angleAccelX(0), angleAccelY(0), angleAccelZ(0), 
      angleFilteredX(0), angleFilteredY(0), angleFilteredZ(0), angleYaw(0), vertical(1500), dt(0.01) {

    
    this->escPin1.attach(escPin1); 
    this->escPin2.attach(escPin2);
    this->escPin3.attach(escPin3);
    this->escPin4.attach(escPin4);
    this->escPin5.attach(escPin5);
    this->escPin6.attach(escPin6);
}

void MPU6050_PID_Controller::setup() {
  Serial.begin(9600);

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // PID Initialization
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-500, 500); // PID output range

  // Initial ESC signal (stable position)
  escPin1.writeMicroseconds(1500);
  escPin2.writeMicroseconds(1500);
  escPin3.writeMicroseconds(1500);
  escPin4.writeMicroseconds(1500);
  escPin5.writeMicroseconds(1500);
  escPin6.writeMicroseconds(1500);

}

void MPU6050_PID_Controller::update() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Angular velocity for all axes
  gyroRateX = gyro.gyro.x;
  gyroRateY = gyro.gyro.y;
  gyroRateZ = gyro.gyro.z;

  // Calculate tilt angles using accelerometer
  angleAccelX = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
  angleAccelY = atan2(accel.acceleration.x, accel.acceleration.z) * 180 / PI;
  angleAccelZ = atan2(accel.acceleration.x, accel.acceleration.y) * 180 / PI;

  // calulation of yaw from gyroscope 
  angleYaw += gyroRateZ * dt;

  // Filter angles
  filterAngles();

  // Average the angles from the three axes
  input = (angleFilteredX + angleFilteredY + angleYaw) / 3.0;

  // Run PID to compute control signal
  myPID.Compute();
  
  // vertical value according to inputs : to control the movement of ROV up and down
  vertical = map(joystickVerticalInput , 0 , 1023 , 1300 , 1700);

  // Send control signal to ESC
  sendToESC();

  // Print data for debugging
  printData();
}

void MPU6050_PID_Controller::filterAngles() {
  angleFilteredX = 0.98 * (angleFilteredX + gyroRateX * dt) + 0.02 * angleAccelX;
  angleFilteredY = 0.98 * (angleFilteredY + gyroRateY * dt) + 0.02 * angleAccelY;
  angleFilteredZ = 0.98 * (angleFilteredZ + gyroRateZ * dt) + 0.02 * angleAccelZ; 

  // yaw filteration 
  angelYaw = 0.98 * (angelYaw + gyroRateZ * dt) + 0.02 * angleAccelZ;
}

void MPU6050_PID_Controller::sendToESC() {
   // Map PID output to ESC signal range
  int pwmFrontLeft = map(output + angleFilteredX + angleFilteredY + angleYaw, -500, 500, 1000, 2000);
  int pwmFrontRight = map(output - angleFilteredX + angleFilteredY - angleYaw, -500, 500, 1000, 2000);
  int pwmRearLeft = map(output + angleFilteredX - angleFilteredY + angleYaw, -500, 500, 1000, 2000);
  int pwmRearRight = map(output - angleFilteredX - angleFilteredY - angleYaw, -500, 500, 1000, 2000);

  // vertical motors
  int pwmVertical1 = vertical;
  int pwmVertical2 = vertical;

  // send PWM signals to motors
  escPin1.writeMicroseconds(pwmFrontLeft);    // front left motor
  escPin2.writeMicroseconds(pwmFrontRight);   // front right motor
  escPin3.writeMicroseconds(pwmRearLeft);     // rear left motor
  escPin4.writeMicroseconds(pwmRearRight);    // rear right motor
  escPin5.writeMicroseconds(pwmVertical1); // first vertical motor
  escPin6.writeMicroseconds(pwmVertical1);// second vertical motor
}

void MPU6050_PID_Controller::printData() {
  Serial.print("Gyro (X/Y/Z): ");
  Serial.print(gyroRateX); Serial.print(" / ");
  Serial.print(gyroRateY); Serial.print(" / ");
  Serial.print(gyroRateZ); Serial.print(" | ");

  Serial.print("Tilt (Filtered X/Y/Z): ");
  Serial.print(angleFilteredX); Serial.print(" / ");
  Serial.print(angleFilteredY); Serial.print(" / ");
  Serial.println(angleFilteredZ);

  Serial.print("Yaw (Filtered): ");
  Serial.println(angleYaw);  

  Serial.print("\tPWM Signal : ");
  Serial.println(map(output, -500, 500, 1000, 2000));
}
