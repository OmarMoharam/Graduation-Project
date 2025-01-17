#include <Wire.h>
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "Joystick.h"

// USB and Joystick setup
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

// Joystick inputs
int Xval = 0, Yval = 0, Twist = 0, Slider = 0, Hat = 0, Button = 0;

// Camera Inputs
#define cameraControlX A0
#define cameraControlY A1

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C as master

  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
    while (1); // Halt
  }
  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    Serial.println("Failed to set report parser.");
}

void loop() {
  Usb.Task();
  JoyEvents.GetValues(Xval, Yval, Hat, Twist, Slider, Button);

  // Map joystick inputs (-100 to 100 range for easy computation)
  int surge = constrain(map(Yval, 0, 1023, -100, 100), -100, 100);
  int sway = constrain(map(Xval, 0, 1023, -100, 100), -100, 100);
  int yaw = constrain(map(Twist, 0, 255, -100, 100), -100, 100);
  int heave = constrain(map(Slider, 0, 255, -100, 100), -100, 100);

  int pitch = (Hat == 0) ? 100 : (Hat == 4) ? -100 : 0; // Up/Down
  int roll = (Hat == 6) ? 100 : (Hat == 2) ? -100 : 0; // Left/Right

  int cameraValueX = analogRead(cameraControlX);
  int cameraValueY = analogRead(cameraControlY);

  // Send data via I2C
  Wire.beginTransmission(8); // Address of ROV Arduino
  Wire.write(surge);
  Wire.write(sway);
  Wire.write(yaw);
  Wire.write(heave);
  Wire.write(pitch);
  Wire.write(roll);
  Wire.write(Button);
  Wire.write(cameraValueX);
  Wire.write(cameraValueY);
  uint8_t error = Wire.endTransmission();

  // Check for I2C errors
  if (error == 0) {
    Serial.println("Data sent successfully.");
  } else {
    Serial.print("I2C Error: ");
    Serial.println(error);
  }

  // Optional: Wait for acknowledgment (1 byte)
  Wire.requestFrom(8, 1); // Request 1 byte from ROV Arduino
  if (Wire.available()) {
    uint8_t ack = Wire.read();
    if (ack == 1) {
      Serial.println("ACK received from ROV.");
    } else {
      Serial.println("ACK not received or invalid.");
    }
  } else {
    Serial.println("No ACK received.");
  }

  delay(50); // Refresh rate
}
