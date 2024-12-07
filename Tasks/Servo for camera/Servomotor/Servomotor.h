#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <stdint.h>
#include <Servo.h>

void resetPosition();
void getAngle();
void updateServoAngles();


extern Servo servoX; 
extern Servo servoY;

extern uint16_t joystickX; 
extern uint16_t joystickY;
extern uint16_t  buttonPin;  

extern uint16_t angleX;    
extern uint16_t angleY;   
extern const uint16_t  minAngle;
extern const uint16_t  maxAngle;


#endif 