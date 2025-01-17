#include "UnderwaterProject.h"
#include "PID.h"

// MPU6050 + Magnetometer Setup
MPU6050_Sensor mpu;
HMC5883L_Sensor hmc;
float rollReading = 0, pitchReading = 0, yawReading = 0;
float alpha = 0.98; // Complementary filter coefficient

// Current Sensor Setup
ACS712 current_sensor(CURRENT_PIN, INPUT_VOLTAGE, ARDUINO_ADC, SENSOR_SCALE_FACTOR);
unsigned long current_u32PreviousMillis = 0;

// Pressure and Depth Sensor Setup
unsigned long pressure_u32PreviousMillis = 0;
const int clock = 8; // Changed to avoid pin conflict

// Temperature Sensor Setup
TemperatureSensor tempSensor(TEMPERATURE_BUS, TEMPERATURE_RESOLUTION);
unsigned long temperature_u32PreviousMillis = 0;

// Thruster Setup
Servo Thruster1, Thruster2, Thruster3, Thruster4, Thruster5, Thruster6;
int T1 = 0, T2 = 0, T3 = 0, T4 = 0, T5 = 0, T6 = 0;

// Joystick Variables
int groundStationAddress = 8;
int surge = 0, sway = 0, yaw = 0, heave = 0, pitch = 0, roll = 0, Button = 0;
int cameraValueX = 0, cameraValueY = 0;

// Camera Servo Setup
Servo servoX, servoY;
int angleX = 90, angleY = 90;

// Loop timing
unsigned long loopPreviousMillis = 0;
const unsigned long loopInterval = 50; // 50 ms

// Function Implementations
void setupSensorsAndActuators() {
  Serial.begin(115200);

  // MPU6050 + Magnetometer Initialization
  mpu.begin();
  hmc.begin();

  // Current Sensor Initialization
  current_sensor.autoMidPoint(100);
  Serial.print("Current Sensor Midpoint: ");
  Serial.println(current_sensor.getMidPoint());

  // Pressure Sensor Initialization
  SPISetup(clock);

  // Temperature Sensor Initialization
  tempSensor.Temp_voidBegin();
  Serial.print("Parasite power is: ");
  Serial.println(tempSensor.Temp_boolIsParasitePowered() ? "ON" : "OFF");

  // Thruster Initialization
  Thruster1.attach(9);
  Thruster2.attach(10);
  Thruster3.attach(11);
  Thruster4.attach(12);
  Thruster5.attach(13);
  Thruster6.attach(14);
  setThrustersNeutral();

  // Camera Servo Initialization
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(angleX);
  servoY.write(angleY);
  Serial.println("Camera Servo Control Initialized");

  // Lighting Initialization
  pinMode(LIGHTS_PIN, OUTPUT);
  digitalWrite(LIGHTS_PIN, LOW); // Lights off by default
}

void pollJoystick() {
  Wire.beginTransmission(groundStationAddress);
  Wire.write(1); // Request joystick data
  Wire.endTransmission();

  Wire.requestFrom(groundStationAddress, 9);
  if (Wire.available() == 9) {
    surge = Wire.read();
    sway = Wire.read();
    yaw = Wire.read();
    heave = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
    Button = Wire.read();
    cameraValueX = Wire.read();
    cameraValueY = Wire.read();
  } else {
    Serial.println("Error: Incomplete joystick data received.");
    surge = sway = yaw = heave = pitch = roll = Button = cameraValueX = cameraValueY = 0;
  }
}

void calculateThrusters() {
  T1 = surge + sway + yaw;
  T2 = surge - sway - yaw;
  T3 = -surge + sway + yaw;
  T4 = -surge - sway - yaw;
  T5 = heave + pitch;
  T6 = heave - pitch;
}

void setThrustersNeutral() {
  Thruster1.writeMicroseconds(1500);
  Thruster2.writeMicroseconds(1500);
  Thruster3.writeMicroseconds(1500);
  Thruster4.writeMicroseconds(1500);
  Thruster5.writeMicroseconds(1500);
  Thruster6.writeMicroseconds(1500);
}

void controlLighting() {
  if (Button == 1) {
    digitalWrite(LIGHTS_PIN, HIGH);
    Serial.println("Light ON");
  } else {
    digitalWrite(LIGHTS_PIN, LOW);
    Serial.println("Light OFF");
  }
}

void updateIMUData() {
  // Update sensor readings
  mpu.update();
  hmc.update();

  // Get raw sensor data
  float rawRoll = mpu.getRoll();
  float rawPitch = mpu.getPitch();
  float rawYaw = hmc.getYaw();

  // Apply Complementary Filter
  rollReading = alpha * rollReading + (1 - alpha) * rawRoll;
  pitchReading = alpha * pitchReading + (1 - alpha) * rawPitch;
  yawReading = alpha * yawReading + (1 - alpha) * rawYaw;

  // Print final roll, pitch, and yaw
  Serial.print("Roll: ");
  Serial.print(rollReading);
  Serial.print("\tPitch: ");
  Serial.print(pitchReading);
  Serial.print("\tYaw: ");
  Serial.println(yawReading);
}
