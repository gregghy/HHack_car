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
import pyttsx3

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

# Set up GPIO
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for TRIG and ECHO
TRIG = 6  # GPIO pin 6 for TRIG
ECHO = 5  # GPIO pin 5 for ECHO

# Set up the trigger and echo pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)



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
speech_running = True



engine = pyttsx3.init()

# RATE
rate = engine.getProperty('rate')   # getting details of current speaking rate
engine.setProperty('rate', 140)

# VOICE
voices = engine.getProperty('voices')       # getting details of current voice
#engine.setProperty('voice', voices[0].id)  # changing index, changes voices. o for male
engine.setProperty('voice', voices[1].id)


# Ultra-fast callback function
def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)

    # Feed small chunks of audio directly into Vosk
    if recognizer.AcceptWaveform(indata.tobytes()):
        result = json.loads(recognizer.Result()).get("text", "")
        if result:
            print(f"\n🔹 Recognized: {result}")
            process_command(result)  # Instant response
    else:
        # Show live partial results
        partial = json.loads(recognizer.PartialResult()).get("partial", "")
        if partial:
            print(f"\r🟡 {partial}", end="", flush=True)  # Overwrites the current line

# Command processing for instant response
def process_command(text):
    global speech_running
    if "stop" in text.lower():
        text = "I will not call nine one one"
        print("\n🛑" + text)
        speak(text)
        speech_running = False
    elif "hello" in text.lower():
        text = "Hello! How can I help?"
        print("\n👋 " + text)
        speak(text)
    elif "Yes" in text.lower():
        text = "I will call nine one one"
        speak(text)
        
# Threaded function to run speech recognition
def start_recognition():
    global speech_running
    print("\n🎤 Listening... (Say 'stop' to exit)")
    
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype="int16",
                        callback=callback, blocksize=BUFFER_SIZE):
        while speech_running:
            sd.sleep(100)

def speak(text):
    engine = pyttsx3.init()
    engine.setProperty('rate', 140)
    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[1].id)  # Adjust if needed
    engine.say(text)
    engine.runAndWait()

"""
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
    distance = round(elapsed_time * 17150, 2)  # Speed of sound is 343 m/s

    return distance
"""
def get_distance():
    # Ensure the trigger is low to start
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.1)

    # Send a pulse to the TRIG pin
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)

    # Wait for the ECHO pin to go HIGH (signal sent)
    pulse_start = time.time()
    while GPIO.input(ECHO) == GPIO.LOW:
        pulse_start = time.time()
        if pulse_start - time.time() > 0.1:  # Timeout after 100ms
            print("Error: Timeout waiting for echo start")
            return -1  # Indicate error if timeout occurs

    # Wait for the ECHO pin to go LOW (signal received)
    pulse_end = time.time()
    while GPIO.input(ECHO) == GPIO.HIGH:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.1:  # Timeout after 100ms
            print("Error: Timeout waiting for echo end")
            return -1  # Indicate error if timeout occurs

    # Calculate the pulse duration
    pulse_duration = pulse_end - pulse_start
    print(f"Raw pulse duration: {pulse_duration} seconds")  # Debugging line

    # Calculate distance in cm using the formula: distance = (pulse_duration * speed_of_sound) / 2
    distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s (divided by 2 to account for round-trip)
    print(f"Raw distance calculation: {distance} cm")  # Debugging line
    return round(distance, 2)

# Function to control the car's movement
def move_car(direction, distance):
    if distance < 80:  # Stop if an obstacle is too close
        print("Obstacle detected! Stopping car.")
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)
        return

    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    
    if direction == "backward":
        GPIO.output(front_left_motor_forward, GPIO.LOW)
        GPIO.output(front_left_motor_backward, GPIO.HIGH)
        GPIO.output(front_right_motor_forward, GPIO.HIGH)
        GPIO.output(front_right_motor_backward, GPIO.LOW)
    elif direction == "foward":
        GPIO.output(front_left_motor_forward, GPIO.HIGH)
        GPIO.output(front_left_motor_backward, GPIO.LOW)
        GPIO.output(front_right_motor_forward, GPIO.LOW)
        GPIO.output(front_right_motor_backward, GPIO.HIGH)
    elif direction == "left":
        GPIO.output(front_left_motor_forward, GPIO.HIGH)
        GPIO.output(front_left_motor_backward, GPIO.LOW)
        GPIO.output(front_right_motor_forward, GPIO.HIGH)
        GPIO.output(front_right_motor_backward, GPIO.LOW)
    elif direction == "right":
        GPIO.output(front_left_motor_forward, GPIO.LOW)
        GPIO.output(front_left_motor_backward, GPIO.HIGH)
        GPIO.output(front_right_motor_forward, GPIO.LOW)
        GPIO.output(front_right_motor_backward, GPIO.HIGH)
    else:
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)

def stop_car():
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


def detect_fall(landmarks, height):
    """
    Detects if a person has fallen based on pose landmarks.
    - Checks if the head and hips are at a low height relative to shoulders.
    """
    if landmarks is None:
        return False

    # Get key landmark positions
    nose_y = landmarks[mp_pose.PoseLandmark.NOSE].y * height
    left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height
    right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height
    left_hip_y = landmarks[mp_pose.PoseLandmark.LEFT_HIP].y * height
    right_hip_y = landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y * height

    
    # Average shoulder and hip height
    avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2
    avg_hip_y = (left_hip_y + right_hip_y) / 2

    """
    # Fall detection logic: If the head and hips are near the ground (below shoulders)
    if nose_y > avg_shoulder_y and avg_hip_y > avg_shoulder_y:
        return True  # Fall detected
    """
    if nose_y > left_shoulder_y and nose_y > left_hip_y:
        return True
    if nose_y > right_shoulder_y and nose_y > right_hip_y:
        return True
    if avg_hip_y <= avg_shoulder_y+20:
        return True

    return False  # No fall detected


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
        distance = get_distance()
        print(distance)
        move_car(person_direction, distance)
        color = (0, 0, 255)
        #move_to_person(fall_detected, text, color)
        
        if distance <= 80:
            stop_car()
            text = "You are on the ground. Should I call nine one one?"
            speak(text)
            # Run recognition in a high-priority thread
            recognition_thread = threading.Thread(target=start_recognition, daemon=True)
            recognition_thread.start()

            # Keep script running
            while speech_running:
                pass
    else:
        stop_car()
        text = "Inspecting"
        color = (0, 255, 0)

        #    Display the direction on the screen
    cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    cv2.imshow("Car Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
GPIO.cleanup()
engine.stop()
cv2.destroyAllWindows()

