#include <Wire.h>
#include <Servo.h>

// Thruster setup
Servo Thruster1; // Forward-right (45°)
Servo Thruster2; // Forward-left (135°)
Servo Thruster3; // Backward-right (315°)
Servo Thruster4; // Backward-left (225°)
Servo Thruster5; // Vertical-right
Servo Thruster6; // Vertical-left

int surge = 0, sway = 0, yaw = 0, heave = 0, pitch = 0, roll = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8); // Initialize I2C as slave with address 8
  Wire.onReceive(receiveData);

  // Attach thrusters
  Thruster1.attach(9);  // PWM pin
  Thruster2.attach(10);
  Thruster3.attach(11);
  Thruster4.attach(12);
  Thruster5.attach(13);
  Thruster6.attach(14);

  // Neutral positions
  setThrustersNeutral();
}

void loop() {
  // Thruster calculations
  int T1 = surge + sway + yaw;  // Forward-right
  int T2 = surge - sway - yaw;  // Forward-left
  int T3 = -surge + sway + yaw; // Backward-right
  int T4 = -surge - sway - yaw; // Backward-left
  int T5 = heave + pitch;       // Vertical-right
  int T6 = heave - pitch;       // Vertical-left

  // Apply thruster outputs
  Thruster1.writeMicroseconds(constrain(map(T1, -100, 100, 1100, 1900), 1100, 1900));
  Thruster2.writeMicroseconds(constrain(map(T2, -100, 100, 1100, 1900), 1100, 1900));
  Thruster3.writeMicroseconds(constrain(map(T3, -100, 100, 1100, 1900), 1100, 1900));
  Thruster4.writeMicroseconds(constrain(map(T4, -100, 100, 1100, 1900), 1100, 1900));
  Thruster5.writeMicroseconds(constrain(map(T5, -100, 100, 1100, 1900), 1100, 1900));
  Thruster6.writeMicroseconds(constrain(map(T6, -100, 100, 1100, 1900), 1100, 1900));

  delay(50); // Refresh rate
}

void setThrustersNeutral() {
  Thruster1.writeMicroseconds(1500); // Neutral signal
  Thruster2.writeMicroseconds(1500);
  Thruster3.writeMicroseconds(1500);
  Thruster4.writeMicroseconds(1500);
  Thruster5.writeMicroseconds(1500);
  Thruster6.writeMicroseconds(1500);
}

void receiveData(int byteCount) {
  if (byteCount >= 6) {
    surge = Wire.read();
    sway = Wire.read();
    yaw = Wire.read();
    heave = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
  }
}
