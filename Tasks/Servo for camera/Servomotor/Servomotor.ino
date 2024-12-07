/**
* Auther : Bemwa Emeil
* Date : 4 / 11 / 2024
* Task Name : Servo for camera 
*/ 


//------------------------------------------ Includes -------------------------------------------//

#include <Servo.h> 
#include "Servomotor.h"

#define SERVO_X 6
#define SERVO_Y 7

//------------------------------------------ Setup ----------------------------------------------//

void setup() {
  servoX.attach(SERVO_X);   
  servoY.attach(SERVO_Y); 
  pinMode(buttonPin, INPUT_PULLUP); 

  /* Initialize the servo by certain angle */
  servoX.write(angleX);
  servoY.write(angleY);

  Serial.begin(9600);
  Serial.println("Camera Servo Control Initialized");
}


//------------------------------------------ Main Loop ------------------------------------------//

void loop() {

  /** make sure of existence of data recieved from Rasbperry Pi
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // reading data till last line 

    
    sscanf(data.c_str(), "X%f Y%f", &xVal, &yVal);
  }*/

  getAngle();
  updateServoAngles();
  Serial.print("X Angle: ");
  Serial.println(angleX);
  Serial.print(" Y Angle: ");
  Serial.println(angleY);
  if (digitalRead(buttonPin) == LOW) {
    resetPosition();
  }

  delay(100); 
}