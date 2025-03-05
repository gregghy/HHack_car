import os
import sounddevice as sd
import vosk
import sys
import json
import numpy as np
import threading
import RPi.GPIO as GPIO
import time
import cv2
import mediapipe as mp

from drive.py import *
from fall_detected.py import *
from speech.py import *

# Set up GPIO
GPIO.setmode(GPIO.BCM)

# Motor Driver Pins
left_motor_forward = 17
left_motor_backward = 18
right_motor_forward = 22
right_motor_backward = 23

# Set GPIO pins as output
GPIO.setup(left_motor_forward, GPIO.OUT)
GPIO.setup(left_motor_backward, GPIO.OUT)
GPIO.setup(right_motor_forward, GPIO.OUT)
GPIO.setup(right_motor_backward, GPIO.OUT)

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils  # For drawing landmarks

# Open webcam
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
frame_rate = 24

# Set up microphone parameters
SAMPLE_RATE = 16000  # Vosk requires 16 kHz
CHANNELS = 1         # Mono channel
BUFFER_SIZE = 4000   # Small buffer for ultra-low latency

# Path to the optimized Vosk model (use a faster one!)
MODEL_PATH = "vosk-model-en-us-0.22-lgraph"  # Faster model with better accuracy
#MODEL_PATH = "vosk-model-en-us-0.42-gigaspeech"
# Check if model exists
if not os.path.exists(MODEL_PATH):
    print(f"Model path {MODEL_PATH} not found!")
    sys.exit(1)

# Load Vosk model and recognizer (do this once to reduce overhead)
model = vosk.Model(MODEL_PATH)
recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)

# Flag to control the loop
running = True

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert frame to RGB for MediaPipe
    results = pose.process(frame_rgb)  # Detect pose

    fall_detected = False
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Draw landmarks on frame
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # Check for fall
        fall_detected = detect_fall(landmarks, height)

    # Display warning if fall is detected
    if fall_detected:
        #cv2.putText(frame, "FALL DETECTED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        person_direction = get_person_direction(results.pose_landmarks.landmark, frame.shape[1], frame.shape[0])
        text = f"FALL DETECTED -> Direction: {person_direction}"

    # Move the car based on the direction
        move_car(person_direction)
        color = (0, 0, 255)
        #move_to_person(fall_detected, text, color)
        if person_found:
            # Run recognition in a high-priority thread
            recognition_thread = threading.Thread(target=start_recognition, daemon=True)
            recognition_thread.start()

            # Keep script running
            while running:
                pass
    else:
        move_car("none")
        text = "Inspecting"
        color = (0, 255, 0)

        #    Display the direction on the screen
    cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    cv2.imshow("Car Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
GPIO.cleanup()
cv2.destroyAllWindows()

