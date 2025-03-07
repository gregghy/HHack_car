import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np
import paho.mqtt.client as mqtt

# MQTT Broker Configuration
BROKER_IP = "localhost"  # or IP of the other Raspberry Pi if not using localhost
BROKER_PORT = 1883
TOPIC = "fall_detection/alert"
# Initialize MQTT Client
client = mqtt.Client()
client.connect(BROKER_IP, BROKER_PORT, 60)

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize Mediapipe Pose Detection
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

def send_mqtt_alert():
    """Sends an MQTT alert to another Raspberry Pi."""
    message = "🚨 Fall Detected! Please check on the person immediately."
    client.publish(TOPIC, message)
    print(f"MQTT Alert Sent: {message}")

def detect_fall(landmarks):
    """Detects falls based on body posture."""
    try:
        shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        hip = landmark[mp_pose.PoseLandmark.LEFT_HIP.value]
        knee = landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value]

        # Calculate angle between shoulder, hip, and knee
        angle = np.arctan2(knee.y - hip.y, knee.x - hip.x) - np.arctan2(shoulder.y - hip.y, shoulder.x - hip.x)
        angle = np.abs(np.degrees(angle))

        # If angle is very low (person lying down), trigger alert
        if angle < 45:
            print("🚨 Fall Detected!")
            send_mqtt_alert()
            return True
    except:
        return False
    return False

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect human pose
    results = pose.process(frame)
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        detect_fall(landmarks)

    # Show video feed
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("Fall Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
