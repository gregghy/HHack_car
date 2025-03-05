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

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils  # For drawing landmarks

# Open webcam
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
frame_rate = 24

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

    # Fall detection logic: If the head and hips are near the ground (below shoulders)
    if nose_y > avg_shoulder_y and avg_hip_y > avg_shoulder_y:
        return True  # Fall detected

    return False  # No fall detected

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
        cv2.putText(frame, "FALL DETECTED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # Show output frame
        cv2.imshow("Fall Detection", frame)
        while fall_detected:
            # Get the direction towards the person
            person_direction = get_person_direction(results.pose_landmarks.landmark, frame.shape[1], frame.shape[0])

            # Move the car based on the direction
            move_car(person_direction)

            #    Display the direction on the screen
            cv2.putText(frame, f"Direction: {person_direction}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Car Camera Feed", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Main loop
"""
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame)

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
"""
cap.release()
GPIO.cleanup()
cv2.destroyAllWindows()
