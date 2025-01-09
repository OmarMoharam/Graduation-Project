// TemperatureSensor.h
#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class TemperatureSensor {
  public:
  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief inilize the temperature object.
  * @param [uint8_t] Copy_uint8tPin take the pin number connected to the arduino.
  * @param [uint8_t] Copy_uint8tResolution take the resolution of the sensor.
  * @return [no return].
  * @note resolution must be 9 or 12 but in case of 12 it will be slower but more accurate.
  * @warning no warnings.
  */
    TemperatureSensor(uint8_t Copy_uint8tPin, uint8_t Copy_uint8tResolution);

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief begin the OneWire Communication and retrieve address of the temperature sensor.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    void Temp_voidBegin();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief Set the resolution of the sensor.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note 9 or 12.
  * @warning no warnings.
  */
    void Temp_voidSetResolution();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief check parasite mode or not.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    bool Temp_boolIsParasitePowered();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief print temperature in C and F.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    void Temp_voidPrintTemperature();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief get temperature value in C.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    float Temp_floatGetTemperatureC();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief get temperature value in F.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    float Temp_floatGetTemperatureF();

  /**
  * @author Omar Tarek - V1.0 - Temperature Sensor - 3/11/2024.
  * @brief print address of the sensor in OneWire Bus.
  * @param [void].
  * @param [void].
  * @return [void].
  * @note no note.
  * @warning no warnings.
  */
    void Temp_voidPrintAddress();

  private:
    uint8_t _pin;
    uint8_t _resolution;
    OneWire _oneWire;
    DallasTemperature _tempSensor;
    DeviceAddress _tempAddress;
};

#endif
