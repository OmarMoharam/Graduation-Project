#include <Wire.h>
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "Joystick.h"

// USB and Joystick setup
USB Usb; // USB object for communication
USBHub Hub(&Usb); // USB hub for connecting multiple devices
HIDUniversal Hid(&Usb); // HID (Human Interface Device) for USB communication
JoystickEvents JoyEvents; // Joystick event handler
JoystickReportParser Joy(&JoyEvents); // Joystick report parser

// Joystick inputs
int Xval = 0, Yval = 0, Twist = 0, Slider = 0, Hat = 0, Button = 0;

// Camera Inputs
#define cameraControlX A0 // Analog pin for camera X-axis control
#define cameraControlY A1 // Analog pin for camera Y-axis control

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  Wire.begin(); // Initialize I2C as master

  // Initialize USB communication
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start."); // Error message if USB initialization fails
    while (1); // Halt the program if USB initialization fails
  }
  delay(200); // Short delay to allow USB to stabilize

  // Set up the HID report parser for the joystick
  if (!Hid.SetReportParser(0, &Joy)) {
    Serial.println("Failed to set report parser."); // Error message if parser setup fails
  }
}

void loop() {
  Usb.Task(); // Process USB tasks
  JoyEvents.GetValues(Xval, Yval, Hat, Twist, Slider, Button); // Read joystick values

  // Map joystick inputs to a range of -100 to 100 for easy computation
  int surge = constrain(map(Yval, 0, 1023, -100, 100), -100, 100); // Surge (forward/backward)
  int sway = constrain(map(Xval, 0, 1023, -100, 100), -100, 100); // Sway (left/right)
  int yaw = constrain(map(Twist, 0, 255, -100, 100), -100, 100); // Yaw (rotation)
  int heave = constrain(map(Slider, 0, 255, -100, 100), -100, 100); // Heave (up/down)

  // Map hat switch values to pitch and roll
  int pitch = (Hat == 0) ? 100 : (Hat == 4) ? -100 : 0; // Pitch (up/down)
  int roll = (Hat == 6) ? 100 : (Hat == 2) ? -100 : 0; // Roll (left/right)


  // Send data via I2C to the ROV Arduino (address 8)
  Wire.beginTransmission(8); // Start I2C transmission to device with address 8
  Wire.write(surge); // Send surge value
  Wire.write(sway); // Send sway value
  Wire.write(yaw); // Send yaw value
  Wire.write(heave); // Send heave value
  Wire.write(pitch); // Send pitch value
  Wire.write(roll); // Send roll value
  Wire.write(Button); // Send button state

  uint8_t error = Wire.endTransmission(); // End I2C transmission and check for errors

  // Check for I2C errors
  if (error == 0) {
    Serial.println("Data sent successfully."); // Success message
  } else {
    Serial.print("I2C Error: "); // Error message
    Serial.println(error);
  }

  // Optional: Wait for acknowledgment (1 byte) from the ROV Arduino
  Wire.requestFrom(8, 1); // Request 1 byte from device with address 8
  if (Wire.available()) {
    uint8_t ack = Wire.read(); // Read the acknowledgment byte
    if (ack == 1) {
      Serial.println("ACK received from ROV."); // Success message
    } else {
      Serial.println("ACK not received or invalid."); // Error message
    }
  } else {
    Serial.println("No ACK received."); // Error message if no acknowledgment is received
  }

  delay(50); // Delay to control the refresh rate (20 Hz)
}