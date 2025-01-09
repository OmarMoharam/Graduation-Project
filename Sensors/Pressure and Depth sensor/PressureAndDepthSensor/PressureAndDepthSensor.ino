// include library:
#include "PressureAndDepthSensor.h"

// generate a MCKL signal pin
 const int clock = 9;

void setup() {
  Serial.begin(9600);
  SPISetup(clock);
}

void loop() 
{
  clockSetup(clock);

  Serial.print("Pressure in mbar = ");
  Serial.println(getPressureinMBAR());

  Serial.print("Pressure in mmHg = ");
  Serial.println(getPressureinMMHG());
  
  Serial.print("Depth in m = ");
  Serial.println(getDepthinMeter(getPressureinMBAR()));

  Serial.print("Temperature in C = ");
  Serial.println(getTemperatureinC());

  delay(5000);
}