import RPi.GPIO as GPIO
import time
import cv2
import mediapipe as mp

# Set up GPIO
GPIO.setmode(GPIO.BCM)

# Motor pins setup
front_left_motor_forward = 17
front_left_motor_backward = 18
front_right_motor_forward = 27
front_right_motor_backward = 22
rear_left_motor_forward = 23
rear_left_motor_backward = 24
rear_right_motor_forward = 25
rear_right_motor_backward = 8
ENA = 12
ENB = 13

motor_pins = [front_left_motor_forward, front_left_motor_backward, front_right_motor_forward,
              front_right_motor_backward, rear_left_motor_forward, rear_left_motor_backward,
              rear_right_motor_forward, rear_right_motor_backward, ENA, ENB]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# HC-SR04 Ultrasonic Sensor Pins
TRIG = 5
ECHO = 6
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Function to measure distance
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 343 m/s

    return distance

# Function to control the car's movement
def move_car(direction, distance):
    if distance < 80:  # Stop if an obstacle is too close
        print("Obstacle detected! Stopping car.")
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)
        return

    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    
    if direction == "right":
        GPIO.output(front_left_motor_forward, GPIO.LOW)
        GPIO.output(front_left_motor_backward, GPIO.HIGH)
        GPIO.output(front_right_motor_forward, GPIO.HIGH)
        GPIO.output(front_right_motor_backward, GPIO.LOW)
    elif direction == "left":
        GPIO.output(front_left_motor_forward, GPIO.HIGH)
        GPIO.output(front_left_motor_backward, GPIO.LOW)
        GPIO.output(front_right_motor_forward, GPIO.LOW)
        GPIO.output(front_right_motor_backward, GPIO.HIGH)
    elif direction == "forward":
        GPIO.output(front_left_motor_forward, GPIO.HIGH)
        GPIO.output(front_left_motor_backward, GPIO.LOW)
        GPIO.output(front_right_motor_forward, GPIO.HIGH)
        GPIO.output(front_right_motor_backward, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(front_left_motor_forward, GPIO.LOW)
        GPIO.output(front_left_motor_backward, GPIO.HIGH)
        GPIO.output(front_right_motor_forward, GPIO.LOW)
        GPIO.output(front_right_motor_backward, GPIO.HIGH)
    else:
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)


# Function to detect the direction of the person
def get_person_direction(landmarks, frame_width, frame_height):
    if landmarks:
        nose = landmarks[mp_pose.PoseLandmark.NOSE]
        nose_x = int(nose.x * frame_width)
        nose_y = int(nose.y * frame_height)

        # Direction towards the person (left, right, forward)
        if nose_x < frame_width // 3:
            return "left"
        elif nose_x > frame_width * 2 // 3:
            return "right"
        else:
            return "forward"
    return "no_person"
