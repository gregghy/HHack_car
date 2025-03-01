import RPi.GPIO as GPIO
import time
import cv2
import mediapipe as mp

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

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Function to control the car's movement
def move_car(direction):
    if direction == "left":
        GPIO.output(left_motor_forward, GPIO.LOW)
        GPIO.output(left_motor_backward, GPIO.HIGH)
        GPIO.output(right_motor_forward, GPIO.HIGH)
        GPIO.output(right_motor_backward, GPIO.LOW)
    elif direction == "right":
        GPIO.output(left_motor_forward, GPIO.HIGH)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.LOW)
        GPIO.output(right_motor_backward, GPIO.HIGH)
    elif direction == "forward":
        GPIO.output(left_motor_forward, GPIO.HIGH)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.HIGH)
        GPIO.output(right_motor_backward, GPIO.LOW)
    else:
        GPIO.output(left_motor_forward, GPIO.LOW)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.LOW)
        GPIO.output(right_motor_backward, GPIO.LOW)

# Initialize OpenCV
cap = cv2.VideoCapture(0)

# Main loop
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)

    # Draw landmarks on the frame
    if results.pose_landmarks:
        mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # Get the direction towards the person
    person_direction = get_person_direction(results.pose_landmarks.landmark, frame.shape[1], frame.shape[0])

    # Move the car based on the direction
    move_car(person_direction)

    # Display the direction on the screen
    cv2.putText(frame, f"Direction: {person_direction}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Car Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
GPIO.cleanup()
cv2.destroyAllWindows()

