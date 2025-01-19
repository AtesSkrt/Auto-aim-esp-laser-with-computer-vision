# Auto-aim-esp-laser-with-computer-vision
2 servo motors and a laser pointer. auto detects your hand with computer vision and aims to it. I hope us army is not planning to use such technologies in the Middle East.


# Laser Tracking System

This project uses a Logitech Brio 100 webcam, ESP8266, two servo motors, and a laser to track and point the laser at a detected hand's palm in real-time. It leverages **MediaPipe Hands** for palm detection and communicates with the ESP8266 via serial connection to control the servo motors.

---

## Demonstration
Check out the video demonstration of this project on YouTube:
https://youtu.be/lr0-SiXOJu4

[![Laser Tracking System - YouTube Video](https://img.youtube.com/vi/lr0-SiXOJu4/0.jpg)](https://youtu.be/lr0-SiXOJu4)

---

## Features
- Real-time hand palm detection using MediaPipe.
- Laser pointer alignment with the palm using two servo motors.
- Interactive offset calibration using `W`, `A`, `S`, `D` keys.
- Adjustable field of view and camera-laser offset.
- Serial communication for precise motor control.

---

## Hardware Requirements
- **Logitech Brio 100** or any webcam with a 58° diagonal field of view.
- **ESP8266** microcontroller.
- **2 Servo Motors** (X and Y axes).
- **Laser Diode**.
- Power supply for the components.
- USB cable for ESP8266.

---

## Software Requirements
- Python 3.7+
- Arduino IDE
- Libraries:
  - OpenCV
  - MediaPipe
  - PySerial

Install Python libraries:
```bash
pip install opencv-python mediapipe pyserial
```

---

## Hardware Setup
1. Connect the components as follows:
   - **Servo X**:
     - Signal → GPIO4 (D2) on ESP8266
     - Power → 5V
     - Ground → GND
   - **Servo Y**:
     - Signal → GPIO5 (D1) on ESP8266
     - Power → 5V
     - Ground → GND
   - **Laser**:
     - Positive → GPIO2 (D4) on ESP8266
     - Negative → GND
2. Position the laser approximately 2-3 cm to the left of the camera lens.
3. Ensure all components are powered correctly.

---

## Code Setup

### ESP8266 Code
Upload the following code to the ESP8266 using the Arduino IDE:

```c
#include <Servo.h>

Servo servoX;
Servo servoY;

// Pin assignments
const int servoXPin = 4; // GPIO4 (D2)
const int servoYPin = 5; // GPIO5 (D1)

void setup() {
  Serial.begin(9600);
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int xAngle = data.substring(0, commaIndex).toInt();
      int yAngle = data.substring(commaIndex + 1).toInt();
      servoX.write(xAngle);
      servoY.write(yAngle);
    }
  }
}
```

### Python Code
Run the provided Python script (`laser_tracking.py`) to detect the palm and control the laser. Use the following key bindings for calibration:
- **`W`**: Decrease `Y_OFFSET`.
- **`S`**: Increase `Y_OFFSET`.
- **`A`**: Decrease `X_OFFSET`.
- **`D`**: Increase `X_OFFSET`.
- **`Q`**: Quit the application.

---

## How to Run
1. Connect the ESP8266 to your computer via USB.
2. Upload the ESP8266 code using the Arduino IDE.
3. Run the Python script:
   ```bash
   python laser_tracking.py
   ```
4. Observe the laser following the detected palm on the webcam feed.
5. Use `W`, `A`, `S`, `D` to adjust the offsets for fine-tuning.

---

## Future Enhancements
- Add Wi-Fi communication between Python and ESP8266.
- Implement multi-hand tracking.
- Enhance precision using advanced mapping techniques.
- Add a GUI for interactive calibration.

---

## License
This project is licensed under the MIT License. See the LICENSE file for details.

---

## Acknowledgments
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html)
- [OpenCV](https://opencv.org/)
- [Logitech Brio 100 Camera](https://www.logitech.com/)

---

## Issues
For issues or contributions, feel free to open a pull request or an issue on this repository.
