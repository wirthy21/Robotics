import time
import cv2
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

class Robot:
    def __init__(self):
        # Initialize motor speed and GPIO pins for motor direction
        self.move_speed = 0x7FFF
        self.LEFT_BACK = 23
        self.LEFT_FRONT = 24
        self.RIGHT_BACK = 27
        self.RIGHT_FRONT = 22

        # Initialize I2C communication and PCA9685 for motor control
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(self.i2c_bus)
        self.pwm.frequency = 60

        # Set up GPIO pins for motor direction
        GPIO.setup(self.LEFT_BACK, GPIO.OUT)
        GPIO.setup(self.LEFT_FRONT, GPIO.OUT)
        GPIO.setup(self.RIGHT_BACK, GPIO.OUT)
        GPIO.setup(self.RIGHT_FRONT, GPIO.OUT)

        # Define PWM channels for motor speed control
        self.ENA = 0
        self.ENB = 1

    # Configure GPIO settings
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    def changespeed(self, leftSpeed, rightSpeed):
        # Set PWM duty cycle for left and right motors
        self.pwm.channels[self.ENA].duty_cycle = leftSpeed
        self.pwm.channels[self.ENB].duty_cycle = rightSpeed

    def stopcar(self):
        # Stop the robot by setting motor direction pins to LOW and speed to 0
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.LOW)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
        self.changespeed(0, 0)

    def backward(self):
        # Move the robot backward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    def forward(self):
        # Move the robot forward
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)

    def turnRight(self):
        # Turn the robot to the right
        GPIO.output(self.LEFT_BACK, GPIO.LOW)
        GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
        GPIO.output(self.RIGHT_BACK, GPIO.HIGH)
        GPIO.output(self.RIGHT_FRONT, GPIO.LOW)

    def turnLeft(self):
        # Turn the robot to the left
        GPIO.output(self.LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.LEFT_FRONT, GPIO.LOW)
        GPIO.output(self.RIGHT_BACK, GPIO.LOW)
        GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)
