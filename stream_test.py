import cv2
import mediapipe as mp

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils  # For drawing landmarks

# Open webcam
cap = cv2.VideoCapture(0)

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

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
