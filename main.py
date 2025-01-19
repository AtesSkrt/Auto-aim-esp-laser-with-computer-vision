import cv2
import mediapipe as mp
import serial
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)

# Serial Port Configuration
SERIAL_PORT = "/dev/cu.usbserial-21210"  # Replace with your serial port
BAUD_RATE = 9600

# Camera and Servo Calibration Parameters
CAMERA_RESOLUTION = (1920, 1080)  # Resolution of Logitech Brio 100
CAMERA_FOV = 58  # Diagonal FOV in degrees
CAMERA_OFFSET_X = -3  # Laser is placed 2-3 cm left of the camera
SERVO_NEUTRAL_X = 90  # Neutral servo position for X (degrees)
SERVO_NEUTRAL_Y = 90  # Neutral servo position for Y (degrees)
X_OFFSET = 0  # Calibration offset for X-axis
Y_OFFSET = 0  # Calibration offset for Y-axis

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
    time.sleep(2)  # Wait for the connection to stabilize
except Exception as e:
    print(f"Error: Could not open serial port. {e}")
    exit()

# Function to map pixels to servo angles
def map_to_servo_angle(pixel, resolution, fov, neutral_angle, offset):
    angle_per_pixel = fov / resolution
    offset_from_center = pixel - (resolution / 2)
    return int(neutral_angle + (offset_from_center * angle_per_pixel) + offset)

# Start capturing from the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])

if not cap.isOpened():
    print("Error: Could not access the webcam.")
    exit()

print("Press 'q' to quit. Use W/A/S/D to adjust offsets (W/S for Y, A/D for X).")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture a frame.")
        break

    # Flip the frame horizontally for a mirror view
    frame = cv2.flip(frame, 1)

    # Convert to RGB (MediaPipe requires RGB input)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame for hand detection
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Get the palm center (wrist landmark)
            palm_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * CAMERA_RESOLUTION[0])
            palm_y = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * CAMERA_RESOLUTION[1])

            # Adjust palm_x for the laser offset from the camera
            palm_x += int(CAMERA_OFFSET_X * CAMERA_RESOLUTION[0] / CAMERA_FOV)

            # Map pixel positions to servo angles
            x_angle = map_to_servo_angle(palm_x, CAMERA_RESOLUTION[0], CAMERA_FOV, SERVO_NEUTRAL_X, X_OFFSET)
            y_angle = map_to_servo_angle(palm_y, CAMERA_RESOLUTION[1], CAMERA_FOV, SERVO_NEUTRAL_Y, Y_OFFSET)

            # Send the angles to the ESP8266 via serial
            command = f"{x_angle},{y_angle}\n"
            ser.write(command.encode())
            print(f"Sent: {command.strip()} | X_OFFSET: {X_OFFSET}, Y_OFFSET: {Y_OFFSET}")

            # Draw a circle on the palm
            cv2.circle(frame, (palm_x, palm_y), 10, (0, 255, 0), -1)

            # Draw hand landmarks for debugging
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display offset values on the frame
    offset_text = f"X_OFFSET: {X_OFFSET}, Y_OFFSET: {Y_OFFSET}"
    cv2.putText(frame, offset_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Laser Calibration", frame)

    # Keyboard controls for interactive calibration
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # Quit
        break
    elif key == ord('a'):  # Decrease X offset
        X_OFFSET -= 1
    elif key == ord('d'):  # Increase X offset
        X_OFFSET += 1
    elif key == ord('w'):  # Decrease Y offset
        Y_OFFSET -= 1
    elif key == ord('s'):  # Increase Y offset
        Y_OFFSET += 1

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
print("Program terminated.")
