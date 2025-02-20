from simple_pid import PID
import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from classes_jannik import Robot
import numpy as np

# Initialize the robot and camera
robot = Robot()
picam2 = Picamera2()
picam2.start()

# Video and robot parameters
frame_rate = 30
video_duration = 120
frame_width, frame_height = 640, 480
turn_speed = 0x5FFF
straight_speed = 0x6FFF
max_speed = 0x7FFF

# Initialize PID controller for line following
pid_direction = PID(Kp=100, Ki=340, Kd=9, setpoint=frame_width // 2)
pid_direction.output_limits = (-max_speed // 2, max_speed // 2)

# Track previous speeds to handle turns
prev_speed_left = 0
prev_speed_right = 0

qr_detector = cv2.QRCodeDetector()

def preprocess_image(frame):
    # Focus on the lower third of the frame to detect the line
    height, width, _ = frame.shape
    roi = frame[int(height * 2 / 3):, :]

    # Convert to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Threshold the image to detect the black line
    _, binary = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Find the largest contour and calculate its centroid
    largest = max(contours, key=cv2.contourArea) if contours else None
    if largest is not None:
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            return cx
    return None

def detect_duck(frame):
    # Convert frame to HSV to detect yellow objects
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    lower_yellow = (20, 100, 100)
    upper_yellow = (30, 255, 255)

    # Create a mask for yellow objects and find contours
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Check if a large yellow object (duck) is detected
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:
            print("Careful, there is a Duck!")
            return True
    return False

def process_qr_code(frame, qr_detector):
    frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=-90)  # Kontrast erhöhen
    frame = cv2.filter2D(frame, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]))  # Schärfen

    data, _, _ = qr_detector.detectAndDecode(frame)
    if data:
        print("QR Code readable:", data)
        robot.stopcar()
        return execute_instruction(data)
        
    print("QR code not readable, using ORB matching...")
    robot.stopcar()
    return match_with_orb(frame)


def match_with_orb(frame):
    orb = cv2.ORB_create()
    _, descriptors_image = orb.detectAndCompute(frame, None)

    reference_images = {
        "car_rotate_720": "car_rotate_720.PNG",
        "car_stop_10s": "car_stop_10s.PNG",
        "car_turn_around": "car_turn_around.PNG"}

    best_match, best_score = None, float("inf")
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    for label, path in reference_images.items():
        ref_img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        _, descriptors_ref = orb.detectAndCompute(ref_img, None)
        matches = bf.match(descriptors_ref, descriptors_image)
        if matches:
            avg_distance = np.mean([m.distance for m in sorted(matches, key=lambda x: x.distance)[:10]])
            if avg_distance < best_score:
                best_score, best_match = avg_distance, label

    if best_match:
        print("Best ORB match found:", best_match)
        return execute_instruction(best_match)

    print("No match found.")
    return False

def execute_instruction(data):
    print("Executing instruction:", data)

    if data == "car_stop_10s":
        picam2.stop()
        data = None
        time.sleep(10)
        picam2.start()
        return True
    elif data == "car_turn_around":
        qr_turn_speed = 0x5FFF
        start_time_turn = time.time()
        picam2.stop()
        data = None
        while time.time() - start_time_turn < 1:
            robot.changespeed(qr_turn_speed, qr_turn_speed)
            robot.turnRight()
        picam2.start()    
        return True
    elif data == "car_rotate_720":
        qr_turn_speed = 0x5FFF
        start_time_rotate = time.time()
        picam2.stop()
        data = None
        while time.time() - start_time_rotate < 4:
            robot.changespeed(qr_turn_speed, qr_turn_speed)
            robot.turnRight()
        picam2.start()
        return True
    return False


# Start the main loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture a frame from the camera
        frame = picam2.capture_array()
        
        # Stop if a duck is detected
        if detect_duck(frame) == True:
            robot.stopcar()
            continue
        
        # Process QR codes if detected:
        if qr_detector.detect(frame)[0]:  # True, wenn ein QR-Code erkannt wurde
            print("QR Code detected")
            robot.stopcar()
            process_qr_code(frame, qr_detector)

        # Preprocess the frame to find the centroid of the line
        cx = preprocess_image(frame)

        if cx is not None:
            # Adjust direction using the PID controller
            direction_speed = pid_direction(cx)
            
            # Scale turn speeds for sharp turns
            turn_scale = 1.0
            if cx < 80 or cx > 220:
                turn_scale = 1.2
            
            # Calculate motor speeds
            left_speed = int(max(0, min(max_speed, straight_speed - direction_speed * turn_scale)))
            right_speed = int(max(0, min(max_speed, straight_speed + direction_speed * turn_scale)))
            prev_speed_left = left_speed
            prev_speed_right = right_speed

            # Update the robot's movement
            robot.changespeed(left_speed, right_speed)
            robot.forward()
        else:
            # Stop and decide turning direction if no line is detected
            robot.stopcar()
            robot.changespeed(turn_speed, turn_speed)
            if prev_speed_left - prev_speed_right > -15000:
                robot.turnRight()
            elif prev_speed_left - prev_speed_right < 15000:
                robot.turnLeft()
            else:
                robot.stopcar()

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup resources after loop ends
    picam2.stop()
    cv2.destroyAllWindows()
    robot.stopcar()
    print("Driving done!")
