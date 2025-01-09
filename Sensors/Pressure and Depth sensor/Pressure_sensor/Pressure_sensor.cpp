#include "Pressure_sensor.h"


float pressure_f32PressureReading = 0;
float pressure_f32Depth = 0;
float pressure_f32Pressure = 0;



float pressure_f32CalculateDepth(float pressure_f32PressureReading) {
    pressure_f32Pressure = pressure_f32PressureReading * pressure_f32CalibrationFactor; 
    pressure_f32Depth = (pressure_f32Pressure - pressure_f32SurfacePressure) / (9.8 * 0.0981);
    return pressure_f32Depth;
}






uint16_t pressure_u16readPressureMS5540C() {
    uint16_t pressure_u16pressureData = 0;
    digitalWrite(CS_PIN, LOW); 


    
    SPI.transfer(0x0F);  
    SPI.transfer(0xF0); 

    delay(10);  

  
    pressure_u16pressureData = SPI.transfer(0x00) << 8;
    pressure_u16pressureData |= SPI.transfer(0x00);  

    digitalWrite(CS_PIN, HIGH); 
    return pressure_u16pressureData;
}

