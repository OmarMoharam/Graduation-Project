/**
* Auther : Bemwa Emeil
* Date : 6 / 11 / 2024
* Task Name : transmit a signal to turn on/off the lights 
*/ 

 //------------------------------------------ Includes -------------------------------------------//

#include <stdint.h>



//----------------------------------------------- Setup -----------------------------------------//
int LIGHTS_PIN = 14;

void setup() {
  // Set up the pin as an output

  pinMode(LIGHTS_PIN, OUTPUT);

  // Begin serial communication for command input

  Serial.begin(9600);
}

//------------------------------------------ Main Loop ------------------------------------------//


void loop() {
 // Check if data is available on the serial port
    
    if (Serial.available() > 0) {

    // Read the incoming byte

    u8 Copy_u8Command = Serial.read();
    

    // Check command for turning the light on or off

    if (Copy_u8Command == '1') {
      digitalWrite(LIGHTS_PIN, HIGH); // Turn on light
      Serial.println("Light ON");
    } 
    else if (Copy_u8Command == '0') {
      digitalWrite(LIGHTS_PIN, LOW); // Turn off light
      Serial.println("Light OFF");
    } 
  } 


}


