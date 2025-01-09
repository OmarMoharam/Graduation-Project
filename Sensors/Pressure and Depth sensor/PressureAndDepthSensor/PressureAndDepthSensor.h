#ifndef PRESSURE_AND_DEPTH_SENSOR_H
#define PRESSURE_AND_DEPTH_SENSOR_H

#include <arduino.h>
#include <SPI.h>

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief used to reset the communication state of a sensor communicating over SPI
          this function ensures the sensor starts in a known and clean state for reliable data acquisition.
 * @param [none].
 * @param [none].
 * @return [void].
 * @note no notes.
 * @warning it sould used after every data transfer.
 */
void resetsensor();

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief this to setup SPI Communication protocol.
 * @param [int] Clock Pin.
 * @param [none].
 * @return [void].
 * @note none.
 * @warning none.
 */
void SPISetup(int clockPin);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief Start Communication.
 * @param [int] Clock Pin.
 * @param [none] .
 * @return [void] .
 * @note none.
 * @warning none.
 */
void clockSetup(int clockPin);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get words from PROM.
 * @param [unsigned int *] to return an array with the 4 words without destruction.
 * @param [none] .
 * @return [void] .
 * @note none.
 * @warning none.
 */
void getCallibrationWords(unsigned int *words);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get the stored coefficients.
 * @param [unsigned int *] to return an array with the 6 coefficients without destruction.
 * @param [none] .
 * @return [void] .
 * @note more than 7m the coefficient need to be changed.
 * @warning none.
 */
void getCoefficients(long *coefficients);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get the raw pressure without any Compensation.
 * @param [unsigned int *] to save the pressure in arrays.
 * @param [none] .
 * @return [void] .
 * @note none.
 * @warning none.
 */
void calculateRawPressure(unsigned int *D1);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get the raw temperature without any Compensation.
 * @param [unsigned int *] to save the temperature in arrays.
 * @param [none] .
 * @return [void] .
 * @note none.
 * @warning none.
 */
void calculateRawTemperature(unsigned int *D2);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get the get Compensated Pressure and Temperature with the help of coefficients.
 * @param [float *] to save the values used to get the pressure and temperature in differnet types.
 * @param [none] .
 * @return [void] .
 * @note none.
 * @warning none.
 */
void getCompensatedPressureTemp(float *values);

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get pressure in mbar.
 * @param [none].
 * @param [none] .
 * @return [float] .
 * @note none.
 * @warning none.
 */
float getPressureinMBAR();

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get pressure in mmHg.
 * @param [none].
 * @param [none] .
 * @return [float] .
 * @note none.
 * @warning none.
 */
float getPressureinMMHG();

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get Temperature in Celsius.
 * @param [none].
 * @param [none] .
 * @return [float] .
 * @note none.
 * @warning none.
 */
float getTemperatureinC();

/**
 * @author Omar - 1.0 - Pressure and Depth - 15/12/2024
 * @brief get depth in meter.
 * @param [float] get the pressure in mbar.
 * @param [none] .
 * @return [float] .
 * @note equation used is "pressure = density * gravity * depth".
 * @warning none.
 */
float getDepthinMeter(float pressureInMBar);
#endif