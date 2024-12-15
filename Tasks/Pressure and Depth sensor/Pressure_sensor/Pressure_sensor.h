#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H


#include <stdint.h>
#include <SPI.h>




float pressure_f32CalculateDepth(float pressure_f32PressureReading);
uint16_t pressure_u16readPressureMS5540C();





const uint8_t MCLK_PIN = 9;
const uint8_t CS_PIN = 10;
const uint8_t MOSI_PIN = 11;
const uint8_t MISO_PIN = 12;
const uint8_t SCK_PIN = 13;

const float pressure_f32CalibrationFactor = 1.02;
const float pressure_f32SurfacePressure = 1013.25;
const float pressure_f32CriticalDepth = 10.00;

extern float pressure_f32PressureReading;
extern float pressure_f32Depth;
extern float pressure_f32Pressure;



#endif