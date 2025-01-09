#include "PressureAndDepthSensor.h"

float valuesArray[3] = {0};

void resetsensor() {
  SPI.setDataMode(SPI_MODE0); 
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}

void SPISetup(int clockPin) {
  SPI.begin(); //see SPI library details on arduino.cc for details
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
  pinMode(clockPin, OUTPUT);
  delay(100);
}

void clockSetup(int clockPin) {
  TCCR1B = (TCCR1B & 0xF8) | 1; //generates the MCKL signal
  analogWrite(clockPin, 128);
}

void getCallibrationWords(unsigned int *words) {
  unsigned int inByteWords[4] = {0};
  resetsensor(); //resets the sensor - caution: afterwards mode = SPI_MODE0!

  //Calibration word 1
  SPI.transfer(0x1D); //send first byte of command to get calibration word 1
  SPI.transfer(0x50); //send second byte of command to get calibration word 1
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  words[0] = SPI.transfer(0x00); //send dummy byte to read first byte of word
  words[0] = words[0] << 8; //shift returned byte 
  inByteWords[0] = SPI.transfer(0x00); //send dummy byte to read second byte of word
  words[0] = words[0] | inByteWords[0]; //combine first and second byte of word

  Serial.print("Calibration word 1 = ");
  Serial.print(words[0],HEX);
  Serial.print(" ");  
  Serial.println(words[0]);  

  resetsensor(); //resets the sensor

  //Calibration word 2; see comments on calibration word 1
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1); 
  words[1] = SPI.transfer(0x00);
  words[1] = words[1] <<8;
  inByteWords[1] = SPI.transfer(0x00);
  words[1] = words[1] | inByteWords[1];

  Serial.print("Calibration word 2 = ");
  Serial.print(words[1],HEX);  
  Serial.print(" ");  
  Serial.println(words[1]);  

  resetsensor(); //resets the sensor

  //Calibration word 3; see comments on calibration word 1
  SPI.transfer(0x1D);
  SPI.transfer(0x90); 
  SPI.setDataMode(SPI_MODE1); 
  words[2] = SPI.transfer(0x00);
  words[2] = words[2] <<8;
  inByteWords[2] = SPI.transfer(0x00);
  words[2] = words[2] | inByteWords[2];

  Serial.print("Calibration word 3 = ");
  Serial.print(words[2],HEX);  
  Serial.print(" ");  
  Serial.println(words[2]);  

  resetsensor(); //resets the sensor

  //Calibration word 4; see comments on calibration word 1
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1); 
  words[3] = SPI.transfer(0x00);
  words[3] = words[3] <<8;
  inByteWords[3] = SPI.transfer(0x00);
  words[3] = words[3] | inByteWords[3];
  Serial.print("Calibration word 4 = ");

  Serial.print(words[3],HEX);
  Serial.print(" ");  
  Serial.println(words[3]);
}

void getCoefficients(long *coefficients) {
  unsigned int wordArray[4] = {0};
  getCallibrationWords(wordArray);

  coefficients[0] = (wordArray[0] >> 1) & 0x7FFF;
  coefficients[1] = ((wordArray[2] & 0x003F) << 6) | (wordArray[3] & 0x003F);
  coefficients[2] = (wordArray[3] >> 6) & 0x03FF;
  coefficients[3] = (wordArray[2] >> 6) & 0x03FF;
  coefficients[4] = ((wordArray[0] & 0x0001) << 10) | ((wordArray[1] >> 6) & 0x03FF);
  coefficients[5] = wordArray[1] & 0x003F;

  
  Serial.print("c1 = ");
  Serial.println(coefficients[0]);
  Serial.print("c2 = ");
  Serial.println(coefficients[1]);
  Serial.print("c3 = ");
  Serial.println(coefficients[2]);
  Serial.print("c4 = ");
  Serial.println(coefficients[3]);
  Serial.print("c5 = ");
  Serial.println(coefficients[4]);
  Serial.print("c6 = ");
  Serial.println(coefficients[5]);
}

void calculateRawPressure(unsigned int *D1) {
  resetsensor(); //resets the sensor

  unsigned int presMSB = 0; //first byte of value
  unsigned int presLSB = 0; //last byte of value
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  presMSB = presMSB << 8; //shift first byte
  presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D1[0] = presMSB | presLSB; //combine first and second byte of value

  Serial.print("D1 - Pressure raw = ");
  Serial.println(D1[0]);
}

void calculateRawTemperature(unsigned int *D2) {
  resetsensor(); //resets the sensor  

  unsigned int tempMSB = 0; //first byte of value
  unsigned int tempLSB = 0; //last byte of value
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  tempMSB = tempMSB << 8; //shift first byte
  tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D2[0] = tempMSB | tempLSB; //combine first and second byte of value

  Serial.print("D2 - Temperature raw = ");
  Serial.println(D2[0]); //voila!
}

void getCompensatedPressureTemp(float *values) {
  long coefficientsArray[6] = {0};
  unsigned int rawPressure[1] = {0};
  unsigned int rawTemperature[1] = {0};

  getCoefficients(coefficientsArray);
  calculateRawPressure(rawPressure);
  calculateRawTemperature(rawTemperature);

  const long UT1 = (coefficientsArray[4] << 3) + 20224;
  const long dT = rawTemperature[0] - UT1;
  const long TEMP = 200 + ((dT * (coefficientsArray[5] + 50)) >> 10);
  const long OFF  = (coefficientsArray[1] * 4) + (((coefficientsArray[3] - 512) * dT) >> 12);
  const long SENS = coefficientsArray[0] + ((coefficientsArray[2] * dT) >> 10) + 24576;
  const long X = (SENS * (rawPressure[0] - 7168) >> 14) - OFF;
  long PCOMP = ((X * 10) >> 5) + 2500;

  values[0] = TEMP;
  values[1] = PCOMP;
  
}

float getTemperatureinC() {
  float TempInC;
  getCompensatedPressureTemp(valuesArray);
  TempInC = valuesArray[0]/10;
  return TempInC;
}

float getPressureinMBAR() {
  float pressureInMBar;
  getCompensatedPressureTemp(valuesArray);
  pressureInMBar = valuesArray[1];
  return pressureInMBar;
}

float getPressureinMMHG() {
  float pressureInMMHg;
  getCompensatedPressureTemp(valuesArray);
  pressureInMMHg = valuesArray[1] * 750.06 / 10000;  // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
  return pressureInMMHg;
}

float getDepthinMeter(float pressureInMBar) {
  const float rho = 1025.0; // density for salt water in kg/m^3
  const float g = 9.81;     // acceleration due to gravity in m/s^2
  return pressureInMBar / (rho * g); // Depth in meters
}