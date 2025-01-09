/** 
* Auther : Bemwa Emeil 
* Date : 1 / 11 / 2024
* Task Name : interface with Pressure & Depth sensor and convert its readings into depth measurements and calibrate them for underwater conditions 
* Breif Description of functions : 1) calculateDepth() : convert the sensor readings into pressure by hPa ( 1 hPa = 100 Pa )
*/





//----------------------------------------- Includes -------------------------------------------//


#include <SPI.h>
#include "Pressure_sensor.h"





//---------------------------------------- Setup -----------------------------------------------//

void setup() {
    Serial.begin(9600);
    pinMode(MCLK_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8); 
    digitalWrite(CS_PIN, HIGH);

    Serial.println("Starting depth measurement.....");
}





//----------------------------------------- Main Loop ------------------------------------------//

void loop() {
    
    pressure_f32PressureReading =  pressure_u16readPressureMS5540C();

    
    pressure_f32Depth = pressure_f32CalculateDepth(pressure_f32PressureReading);

    Serial.print("The Current Depth: ");
    Serial.print(pressure_f32Depth);
    Serial.println(" meters");

    
    if (pressure_f32Depth >= pressure_f32CriticalDepth) {
        Serial.println("WARNING!!! : Reaching Critical Depth ");
    }

    delay(1000); 
}
