#include "ACS712.h"

/*
5A  uses 185 mV/A
20A uses 100 mV/A
30A uses  66 mV/A
*/
#define CURRENT_PIN A0
#define INPUT_VOLTAGE 5.0
#define ARDUINO_ADC 1023
#define SENSOR_SCALE_FACTOR 66

/* Timing for Current Sensor using millis */
const int current_u16Interval = 1000;
unsigned long current_u32PreviousMillis = 0;

/* Safest Current for Motors (8 Amp based on hardware team)*/
#define SAFEST_CURRENT 8

/* number of cycles for accuracy */
#define MIDPOINT_CYCLES 100
#define READING_CYCLES 10

ACS712  current_sensor(CURRENT_PIN, INPUT_VOLTAGE, ARDUINO_ADC, SENSOR_SCALE_FACTOR);

void setup()
{
  Serial.begin(9600);

  /* there is an offset value from 2.5 to 2.54 so we take 100 reading and set the midpoint to 0 for accurate reading */
  current_sensor.autoMidPoint(MIDPOINT_CYCLES);
  Serial.println(current_sensor.getMidPoint());
}


void loop()
{
  uint16_t Local_u8DCReading;
  unsigned long current_u32CurrentMillis = millis();

  if (current_u32CurrentMillis - current_u32PreviousMillis >= current_u16Interval) {
    current_u32PreviousMillis = current_u32CurrentMillis;
    Local_u8DCReading = current_sensor.mA_DC(READING_CYCLES); // take the avg of 10 cycles
    Serial.println(Local_u8DCReading);
  }

  if(Local_u8DCReading > SAFEST_CURRENT) {
    //stop motor
  }
}
