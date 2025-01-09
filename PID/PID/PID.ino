#include "PID.h"

MPU6050_PID_Controller controller(9); // ESC connected to pin 9

void setup() {
  controller.setup(); // Initialize MPU6050 and PID controller
}

void loop() {
  controller.update(); // Update readings, calculate PID, and control ESC
  delay(10); // Wait for next loop
}
