#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

long MPU_longTimer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();

  /*  in this section we should measure the offset in normal postion and
      then put the numbers in the setGyroOffsets */
  //mpu6050.calcGyroOffsets(true);
  //delay(50000);
  mpu6050.setGyroOffsets(-5.49 ,0.64, -0.75);
}

void loop() {
  mpu6050.update();

  if(millis() - MPU_longTimer > 1000){
    
    /* Print the temperature */
    //Serial.print("temp : ");
    //Serial.println(mpu6050.getTemp());

    /* Print the acceleration */
    //Serial.print("accX : ");
    //Serial.print(mpu6050.getAccX());
    //Serial.print("\taccY : ");
    //Serial.print(mpu6050.getAccY());
    //Serial.print("\taccZ : ");
    //Serial.println(mpu6050.getAccZ());

    /* Print the angles of gyro in rad/s */
    //Serial.print("gyroX : ");
    //Serial.print(mpu6050.getGyroX());
    //Serial.print("\tgyroY : ");
    //Serial.print(mpu6050.getGyroY());
    //Serial.print("\tgyroZ : ");
    //Serial.println(mpu6050.getGyroZ());

    /* Print the angles of acc in degree */
    //Serial.print("accAngleX : ");
    //Serial.print(mpu6050.getAccAngleX());
    //Serial.print("\taccAngleY : ");
    //Serial.println(mpu6050.getAccAngleY());

    /* Print the angles of gyro in degree */
    //Serial.print("gyroAngleX : ");
    //Serial.print(mpu6050.getGyroAngleX());
    //Serial.print("\tgyroAngleY : ");
    //Serial.print(mpu6050.getGyroAngleY());
    //Serial.print("\tgyroAngleZ : ");
    //Serial.println(mpu6050.getGyroAngleZ());
    
    /* Print the angles after applying Complementary Filter */
    Serial.print("angleX : ");
    Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");
    Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    //Serial.println("=======================================================\n");
    MPU_longTimer = millis();
    
  }

}
