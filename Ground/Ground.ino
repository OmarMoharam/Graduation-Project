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
  int surge = map(Yval, 0, 1023, -100, 100);
  int sway = map(Xval, 0, 1023, -100, 100);
  int yaw = map(Twist, 0, 255, -100, 100);
  int heave = map(Slider, 0, 255, -100, 100);

  int pitch = (Hat == 0) ? 50 : (Hat == 4) ? -50 : 0; // Up/Down
  int roll = (Hat == 6) ? 50 : (Hat == 2) ? -50 : 0; // Left/Right

  // Send data via I2C
  Wire.beginTransmission(8); // Address of ROV Arduino
  Wire.write(surge);
  Wire.write(sway);
  Wire.write(yaw);
  Wire.write(heave);
  Wire.write(pitch);
  Wire.write(roll);
  Wire.endTransmission();

  delay(50); // Refresh rate
}
