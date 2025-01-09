/*Electronic level with MPU6050 by: Omar Wael Morsi*/
/*YouTube: Morsi Hamed  https://www.youtube.com/c/morsihamed */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

void setup(void) {
  
  Serial.begin(115200);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(2 , OUTPUT);
  pinMode(3 , OUTPUT);
  pinMode(4 , OUTPUT);
  pinMode(5 , OUTPUT);
  pinMode(6 , OUTPUT);

}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

int y = a.acceleration.y;
    Serial.print(y);
        Serial.print("  :  ");

  if(y >= 0 && y <= 1){
  digitalWrite(2 , LOW);
  digitalWrite(3 , LOW);
  digitalWrite(4 , LOW);
  digitalWrite(5 , LOW);
  digitalWrite(6 , LOW);
    Serial.println("0led");

    }
else if(y >= 1 && y <= 2){
    digitalWrite(2 , HIGH);
  digitalWrite(3 , LOW);
  digitalWrite(4 , LOW);
  digitalWrite(5 , LOW);
  digitalWrite(6 , LOW);
      Serial.println("1led");

    }

    else if(y >= 2 && y <= 3){
    digitalWrite(2 , HIGH);
  digitalWrite(3 , HIGH);
  digitalWrite(4 , LOW);
  digitalWrite(5 , LOW);
  digitalWrite(6 , LOW);
      Serial.println("2leds");

    }

    else if(y >= 3 && y <= 4){
    digitalWrite(2 , HIGH);
  digitalWrite(3 , HIGH);
  digitalWrite(4 , HIGH);
  digitalWrite(5 , LOW);
  digitalWrite(6 , LOW);
      Serial.println("3leds");

    }

    else if(y >= 4 && y <= 5){
    digitalWrite(2 , HIGH);
  digitalWrite(3 , HIGH);
  digitalWrite(4 , HIGH);
  digitalWrite(5 , HIGH);
  digitalWrite(6 , LOW);
      Serial.println("4leds");

    }
    else if(y >= 6 && y <= 7){
    digitalWrite(2 , HIGH);
  digitalWrite(3 , HIGH);
  digitalWrite(4 , HIGH);
  digitalWrite(5 , HIGH);
  digitalWrite(6 , HIGH);
      Serial.println("5leds");

    }
    
}

/*I hope this was helpful for you please check tutorial video in my youtube channel*/
/*and if you have any question I will be happy answering it :)*/
/* https://www.youtube.com/playlist?list=PLJRh_zZSG_PT0lJAzx0eTCyPwk_obpJjt */
