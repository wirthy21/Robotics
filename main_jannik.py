from simple_pid import PID
import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from classes_jannik import Robot
import functions_jannik as fj
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

# Start the main loop
start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture a frame from the camera
        frame = picam2.capture_array()
        
        # Stop if a duck is detected
        if fj.detect_duck(frame) == True:
            Robot.stopcar()
            continue
        
        # Process QR codes if detected
        if fj.process_qr_code(frame):
            continue

        # Preprocess the frame to find the centroid of the line
        cx = fj.preprocess_image(frame)

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
            Robot.changespeed(left_speed, right_speed)
            Robot.forward()
        else:
            # Stop and decide turning direction if no line is detected
            Robot.stopcar()
            Robot.changespeed(turn_speed, turn_speed)
            if prev_speed_left - prev_speed_right > -15000:
                Robot.turnRight()
            elif prev_speed_left - prev_speed_right < 15000:
                Robot.turnLeft()
            else:
                Robot.stopcar()

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup resources after loop ends
    picam2.stop()
    cv2.destroyAllWindows()
    Robot.stopcar()
    print("Driving done!")
