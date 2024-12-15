#include "Servomotor.h"
#include <Arduino.h>

Servo servoX;
Servo servoY;

uint16_t joystickX = A0;
uint16_t joystickY = A1;
uint16_t buttonPin = 2;

uint16_t angleX = 90;
uint16_t angleY = 90;
const uint16_t minAngle = 0;
const uint16_t maxAngle = 180;

float xVal;
float yVal;

void resetPosition() {
  angleX = 90;
  angleY = 90;
  servoX.write(angleX);
  servoY.write(angleY);
  Serial.println("Position reset to default (90, 90)");
}

void getAngle() {
  /** to convert the range from ( 1000 : 2000 ) to (-1 : 1 ) , we need some calculations :
 1) minus 1000 from the range to be (0 : 1000)
 2) divided the result by 500 to get the range (0:2)
 3) minus the result by 1 to finally get the range of (-1 : 1)
 */
  xVal = (analogRead(joystickX) / 1023.0) * 2 - 1 ;
  yVal = (analogRead(joystickY) / 1023.0) * 2 - 1 ;

  /* mapping the angles from -100 to 0 and from 100 to 180 */
  angleX = map(xVal * 100 , -100 , 100 , minAngle, maxAngle);
  angleY = map(yVal * 100 , -100 , 100 , minAngle, maxAngle);
}

void updateServoAngles() {
  servoX.write(angleX);
  servoY.write(angleY);
}