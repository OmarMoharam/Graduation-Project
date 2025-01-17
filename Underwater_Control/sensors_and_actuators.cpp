#include "sensors_and_actuators.h"

// MPU6050 + Magnetometer Setup
MPU6050_Sensor mpu; // MPU6050 sensor object
HMC5883L_Sensor hmc; // Magnetometer object
float rollReading = 0, pitchReading = 0, yawReading = 0; // Filtered roll, pitch, and yaw angles
float alpha = 0.98; // Complementary filter coefficient

// Current Sensor Setup
ACS712 current_sensor(CURRENT_PIN, INPUT_VOLTAGE, ARDUINO_ADC, SENSOR_SCALE_FACTOR); // Current sensor object
unsigned long currentSensorTimestamp = 0; // Timestamp for current sensor readings

// Pressure and Depth Sensor Setup
unsigned long pressureSensorTimestamp = 0; // Timestamp for pressure sensor readings
const int clock = 8; // Clock pin for pressure sensor
int maxDepth = 5, minDepth = 0; // Maximum and minimum depth limits

// Temperature Sensor Setup
TemperatureSensor tempSensor(TEMPERATURE_BUS, TEMPERATURE_RESOLUTION); // Temperature sensor object
unsigned long temperatureSensorTimestamp = 0; // Timestamp for temperature sensor readings

// Thruster Setup
Servo Thruster1, Thruster2, Thruster3, Thruster4, Thruster5, Thruster6; // Thruster servo objects
int T1 = 0, T2 = 0, T3 = 0, T4 = 0, T5 = 0, T6 = 0; // Thruster output values

// Joystick Variables
int groundStationAddress = 8; // I2C address of the ground station
int surge = 0, sway = 0, yaw = 0, heave = 0, pitch = 0, roll = 0, Button = 0; // Joystick input values
int cameraValueX = 0, cameraValueY = 0; // Camera control values

// Camera Servo Setup
Servo servoX, servoY; // Camera servo objects
int angleX = 90, angleY = 90; // Camera servo angles

// Lighting System Setup
bool isLightOn = false; // Lighting system state

// Loop timing
unsigned long loopPreviousMillis = 0; // Timestamp for loop timing
const unsigned long loopInterval = 50; // Loop interval in milliseconds (50 ms)

// PID Controller Setup
double setpointRoll = 0, setpointPitch = 0, setpointYaw = 0; // Desired roll, pitch, and yaw angles
double setpointHeave = 0; // Desired depth
double inputRoll = 0, inputPitch = 0, inputYaw = 0; // Actual roll, pitch, and yaw angles
double inputHeave = 0; // Actual depth
double outputRoll = 0, outputPitch = 0, outputYaw = 0; // PID outputs for roll, pitch, and yaw
double outputHeave = 0; // PID output for depth
double kp = 10.0, ki = 2.0, kd = 10.0; // PID tuning parameters (tune as needed)

// Initialize PID controllers
PID rollPID(&inputRoll, &outputRoll, &setpointRoll, kp, ki, kd, DIRECT); // Roll PID controller
PID pitchPID(&inputPitch, &outputPitch, &setpointPitch, kp, ki, kd, DIRECT); // Pitch PID controller
PID yawPID(&inputYaw, &outputYaw, &setpointYaw, kp, ki, kd, DIRECT); // Yaw PID controller
PID heavePID(&inputHeave, &outputHeave, &setpointHeave, kp, ki, kd, DIRECT); // Heave PID controller

// Function Implementations
void setupSensorsAndActuators() {
  Serial.begin(115200); // Initialize serial communication

  // MPU6050 + Magnetometer Initialization
  mpu.begin(); // Initialize MPU6050
  hmc.begin(); // Initialize magnetometer

  // Current Sensor Initialization
  current_sensor.autoMidPoint(100); // Calibrate current sensor
  Serial.print("Current Sensor Midpoint: ");
  Serial.println(current_sensor.getMidPoint());

  // Pressure Sensor Initialization
  SPISetup(clock); // Initialize pressure sensor

  // Temperature Sensor Initialization
  tempSensor.Temp_voidBegin(); // Initialize temperature sensor
  Serial.print("Parasite power is: ");
  Serial.println(tempSensor.Temp_boolIsParasitePowered() ? "ON" : "OFF");

  // Thruster Initialization
  Thruster1.attach(9); // Attach Thruster1 to pin 9
  Thruster2.attach(10); // Attach Thruster2 to pin 10
  Thruster3.attach(11); // Attach Thruster3 to pin 11
  Thruster4.attach(12); // Attach Thruster4 to pin 12
  Thruster5.attach(13); // Attach Thruster5 to pin 13
  Thruster6.attach(14); // Attach Thruster6 to pin 14
  setThrustersNeutral(); // Set thrusters to neutral position

  // Camera Servo Initialization
  servoX.attach(SERVO_X_PIN); // Attach X-axis camera servo
  servoY.attach(SERVO_Y_PIN); // Attach Y-axis camera servo
  servoX.write(angleX); // Set initial X-axis angle
  servoY.write(angleY); // Set initial Y-axis angle
  Serial.println("Camera Servo Control Initialized");

  // Lighting Initialization
  pinMode(LIGHTS_PIN, OUTPUT); // Set lighting pin as output
  digitalWrite(LIGHTS_PIN, LOW); // Turn off lights by default

  // PID Initialization
  rollPID.SetMode(AUTOMATIC); // Enable roll PID controller
  pitchPID.SetMode(AUTOMATIC); // Enable pitch PID controller
  yawPID.SetMode(AUTOMATIC); // Enable yaw PID controller
  heavePID.SetMode(AUTOMATIC); // Enable heave PID controller
}

void pollJoystick() {
  Wire.beginTransmission(groundStationAddress); // Start I2C transmission to ground station
  Wire.write(1); // Request joystick data
  Wire.endTransmission();

  Wire.requestFrom(groundStationAddress, 9); // Request 9 bytes of data
  if (Wire.available() == 9) {
    surge = Wire.read(); // Read surge input
    sway = Wire.read(); // Read sway input
    yaw = Wire.read(); // Read yaw input
    heave = Wire.read(); // Read heave input
    pitch = Wire.read(); // Read pitch input
    roll = Wire.read(); // Read roll input
    Button = Wire.read(); // Read button input
    cameraValueX = Wire.read(); // Read camera X-axis input
    cameraValueY = Wire.read(); // Read camera Y-axis input

    // Set desired angles and depth from joystick
    setpointRoll = map(roll, -100, 100, -30, 30); // Map roll input to ±30 degrees
    setpointPitch = map(pitch, -100, 100, -30, 30); // Map pitch input to ±30 degrees
    setpointYaw = map(yaw, -100, 100, -180, 180); // Map yaw input to ±180 degrees
    setpointHeave = map(heave, -100, 100, minDepth, maxDepth); // Map heave input to depth range
  } else {
    Serial.println("Error: Incomplete joystick data received."); // Log error
    surge = sway = yaw = heave = pitch = roll = Button = cameraValueX = cameraValueY = 0; // Reset inputs
    setThrustersNeutral(); // Reset thrusters to neutral
  }
}

void calculateThrusters() {
  // Calculate thruster outputs based on joystick inputs and PID outputs
  T1 = surge + sway + outputYaw + outputRoll;
  T2 = surge - sway - outputYaw - outputRoll;
  T3 = -surge + sway + outputYaw + outputRoll;
  T4 = -surge - sway - outputYaw - outputRoll;
  T5 = outputHeave + outputPitch;
  T6 = outputHeave - outputPitch;
}

void setThrustersNeutral() {
  // Set all thrusters to neutral position (1500 µs)
  Thruster1.writeMicroseconds(1500);
  Thruster2.writeMicroseconds(1500);
  Thruster3.writeMicroseconds(1500);
  Thruster4.writeMicroseconds(1500);
  Thruster5.writeMicroseconds(1500);
  Thruster6.writeMicroseconds(1500);
}

void controlLighting() {
  if (Button == 1) {
    digitalWrite(LIGHTS_PIN, HIGH); // Turn on lights
    Serial.println("Light ON");
  } else {
    digitalWrite(LIGHTS_PIN, LOW); // Turn off lights
    Serial.println("Light OFF");
  }
}

void updateIMUData() {
  // Update sensor readings
  mpu.update(); // Update MPU6050 data
  hmc.update(); // Update magnetometer data

  // Get raw sensor data
  float rawRoll = mpu.getRoll(); // Get raw roll angle
  float rawPitch = mpu.getPitch(); // Get raw pitch angle
  float rawYaw = hmc.getYaw(); // Get raw yaw angle

  // Apply Complementary Filter
  rollReading = alpha * rollReading + (1 - alpha) * rawRoll; // Filter roll angle
  pitchReading = alpha * pitchReading + (1 - alpha) * rawPitch; // Filter pitch angle
  yawReading = alpha * yawReading + (1 - alpha) * rawYaw; // Filter yaw angle

  // Set input for PID controllers
  inputRoll = rollReading; // Set roll input for PID
  inputPitch = pitchReading; // Set pitch input for PID
  inputYaw = yawReading; // Set yaw input for PID
  inputHeave = getDepthinMeter(getPressureinMBAR()); // Set depth input for PID

  // Compute PID outputs
  rollPID.Compute(); // Compute roll PID output
  pitchPID.Compute(); // Compute pitch PID output
  yawPID.Compute(); // Compute yaw PID output
  heavePID.Compute(); // Compute heave PID output

  // Print final roll, pitch, and yaw
  Serial.print("Roll: ");
  Serial.print(rollReading);
  Serial.print("\tPitch: ");
  Serial.print(pitchReading);
  Serial.print("\tYaw: ");
  Serial.println(yawReading);
}