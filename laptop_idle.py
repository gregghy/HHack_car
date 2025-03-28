import cv2
import mediapipe as mp
import numpy as np
import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# Digital Map Configuration
MAP_WIDTH = 500
MAP_HEIGHT = 400
# Laptop position (bottom-right corner)
LAPTOP_POSITION = (450, 350)

# MQTT Configuration
BROKER_IP = "localhost"  # Broker on the laptop
BROKER_PORT = 1883
TOPIC_FALL_ALERT = "fall_detection/alert"
TOPIC_FALL_POSITION = "fall_detection/position"

# Initialize MQTT Client
client = mqtt.Client()
try:
    client.connect(BROKER_IP, BROKER_PORT, 60)
    client.loop_start()
    print(f"Connected to MQTT broker at {BROKER_IP}:{BROKER_PORT}")
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")

# Initialize MediaPipe Pose Detection
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create a digital map
digital_map = np.ones((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8) * 255
# Mark laptop position
cv2.circle(digital_map, LAPTOP_POSITION, 10, (0, 0, 255), -1)
# Mark robot starting position (top-left corner)
cv2.circle(digital_map, (50, 50), 10, (255, 0, 0), -1)

# Map the camera view to coordinates on the digital map
# This is a simplified mapping - in reality you would calibrate this
def map_camera_to_coordinates(x_ratio, y_ratio):
    """
    Maps normalized camera coordinates (0-1) to the digital map.
    Returns (x, y) coordinates on the map.
    """
    # Invert x and y to reflect real-world positions
    map_x = int(MAP_WIDTH * (1 - x_ratio))
    map_y = int(MAP_HEIGHT * y_ratio)
    return (map_x, map_y)

def detect_fall(landmarks, height):
    """
    Detects if a person has fallen based on pose landmarks.
    - Checks if the head and hips are at a low height relative to shoulders.
    """
    if landmarks is None:
        return False
    # Get key landmark positions
    nose = landmarks[mp_pose.PoseLandmark.NOSE.value]
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
    right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]

    # Average shoulder and hip height
    avg_shoulder_y = (left_shoulder.y + right_shoulder.y) / 2
    avg_hip_y = (left_hip.y + right_hip.y) / 2
    """
    # Fall detection logic: If the head and hips are near the ground (below shoulders)
    if nose_y > avg_shoulder_y and avg_hip_y > avg_shoulder_y:
        return True  # Fall detected
    """
    position_x = (left_shoulder.x + right_shoulder.x + left_hip.x + right_hip.x) / 4
    position_y = (left_shoulder.y + right_shoulder.y + left_hip.y + right_hip.y) / 4

    coords = map_camera_to_coordinates(position_x, position_y)

    if nose.y > left_shoulder.y and nose.y > left_hip.y:
        return True, coords
    if nose.y > right_shoulder.y and nose.y > right_hip.y:
        return True, coords
    if avg_hip_y <= avg_shoulder_y+20:
        return True, coords

        
    return False, 

def send_mqtt_fall_alert(coords):
    """Sends an MQTT alert with fall coordinates to the robot."""
    message = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "coordinates": {
            "x": coords[0],
            "y": coords[1]
        },
        "status": "FALL_DETECTED"
    }
    
    try:
        client.publish(TOPIC_FALL_POSITION, json.dumps(message))
        print(f"MQTT Alert Sent: Fall detected at coordinates {coords}")
        return True
    except Exception as e:
        print(f"Failed to send MQTT message: {e}")
        return False

# Main loop
print("Starting fall detection and mapping...")
fall_detected = False
prev_fall_time = time.time() - 10  # Initialize to allow immediate detection

try:
    while True:
        ret, frame = cap.read()
        height, width, _ = frame.shape
        if not ret:
            print("Failed to grab frame")
            break

        # Create a copy of the digital map for display
        display_map = digital_map.copy()
        
        # Process frame for pose detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert frame to RGB for MediaPipe
        results = pose.process(frame_rgb)  # Detect pose
        is_fallen = False
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            # Draw landmarks on frame
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            # Check for fall
            is_fallen, coords = detect_fall(landmarks, height)
            
            # Only trigger a new fall alert if enough time has passed since the last one
            current_time = time.time()
            if is_fallen and coords and (current_time - prev_fall_time) > 5:
                fall_detected = True
                prev_fall_time = current_time
                
                # Mark the fall position on the map
                cv2.circle(display_map, coords, 10, (0, 255, 0), -1)
                cv2.line(display_map, LAPTOP_POSITION, coords, (0, 0, 255), 2)
                
                # Send the fall position to the robot
                send_mqtt_fall_alert(coords)
                
                print(f"Fall detected at map coordinates: {coords}")
            
            # Draw current person position on the map (for visualization)
            if not is_fallen and coords:
                cv2.circle(display_map, coords, 5, (255, 0, 255), -1)
        
        # Show the frame and map
        cv2.imshow("Fall Detection", frame)
        cv2.imshow("Digital Map", display_map)
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    print("Resources released")
