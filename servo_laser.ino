#include <Servo.h>

Servo servoX;
Servo servoY;

// Pin assignments
const int servoXPin = 4; // GPIO4 (D2)
const int servoYPin = 5; // GPIO5 (D1)

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Attach servos
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Center the servos
  servoX.write(90);
  servoY.write(90);

  Serial.println("Ready to receive commands...");
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming data
    String data = Serial.readStringUntil('\n');
    data.trim(); // Remove whitespace

    // Split the data into x and y angles
    int delimiterIndex = data.indexOf(',');
    if (delimiterIndex != -1) {
      int xAngle = data.substring(0, delimiterIndex).toInt();
      int yAngle = data.substring(delimiterIndex + 1).toInt();

      // Set the servo positions
      servoX.write(xAngle);
      servoY.write(yAngle);

      // Debug output
      Serial.print("X: ");
      Serial.print(xAngle);
      Serial.print(" | Y: ");
      Serial.println(yAngle);
    }
  }
}
