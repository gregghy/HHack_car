import cv2
import mediapipe as mp
import numpy as np
import json
import time
from datetime import datetime
import bluetooth
import threading
import socket

# Digital Map Configuration
MAP_WIDTH = 500
MAP_HEIGHT = 400
# Laptop position (bottom-right corner)
LAPTOP_POSITION = (450, 350)

# Bluetooth Configuration
# The laptop acts as the server
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"  # Arbitrary UUID for this service
bt_server_sock = None
client_sock = None

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

# Flag to control bluetooth server thread
running = True

def setup_bluetooth_server():
    """Sets up a Bluetooth server socket."""
    global bt_server_sock, client_sock
    
    try:
        bt_server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        bt_server_sock.bind(("", bluetooth.PORT_ANY))
        bt_server_sock.listen(1)
        
        port = bt_server_sock.getsockname()[1]
        
        # Advertise the service
        bluetooth.advertise_service(
            bt_server_sock, 
            "FallDetectionService",
            service_id=uuid,
            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE]
        )
        
        print(f"Waiting for Bluetooth connection on RFCOMM channel {port}")
        
        # Accept connection
        client_sock, client_info = bt_server_sock.accept()
        print(f"Accepted connection from {client_info}")
        
        return True
    except Exception as e:
        print(f"Bluetooth server setup failed: {e}")
        return False

def bluetooth_server_thread():
    """Thread to handle incoming Bluetooth messages."""
    global client_sock, running
    
    if client_sock is None:
        print("No Bluetooth connection established")
        return
    
    try:
        while running:
            # Wait for data from the robot (like status updates)
            try:
                data = client_sock.recv(1024)
                if not data:
                    print("Connection lost")
                    break
                
                message = data.decode('utf-8')
                print(f"Received from robot: {message}")
                
                # Process any status updates or commands from the robot
                try:
                    msg_data = json.loads(message)
                    if msg_data.get("status") == "ARRIVED":
                        print("Robot has arrived at the fall location")
                        # Update the map or UI to indicate arrival
                except json.JSONDecodeError:
                    print(f"Received non-JSON message: {message}")
            
            except bluetooth.btcommon.BluetoothError as e:
                if "timed out" in str(e):
                    # Just a timeout, continue the loop
                    pass
                else:
                    print(f"Bluetooth error: {e}")
                    break
            
            time.sleep(0.1)
    
    except Exception as e:
        print(f"Bluetooth server thread error: {e}")
    finally:
        print("Bluetooth server thread stopped")

# Map the camera view to coordinates on the digital map
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
    Detects falls based on body posture.
    Returns (is_fallen, position) where position is (x, y) coordinates if fallen.
    """
    try:
        nose = landmarks[mp_pose.PoseLandmark.NOSE.value]
        left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
        right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]

        # Calculate average positions
        avg_shoulder_y = (left_shoulder.y + right_shoulder.y) / 2 * height
        avg_hip_y = (left_hip.y + right_hip.y) / 2 * height
        nose_y = nose.y * height

        # Position for mapping (use average position of shoulders and hips)
        position_x = (left_shoulder.x + right_shoulder.x + left_hip.x + right_hip.x) / 4
        position_y = (left_shoulder.y + right_shoulder.y + left_hip.y + right_hip.y) / 4

        # Fall detection logic (similar to your existing logic)
        is_fallen = False
        if nose_y > avg_shoulder_y and nose_y > avg_hip_y:
            is_fallen = True
        elif avg_hip_y <= avg_shoulder_y + 20:
            is_fallen = True

        # Map camera position to map coordinates
        map_coords = map_camera_to_coordinates(position_x, position_y)
        
        return is_fallen, map_coords
    except:
        return False, None

def send_bluetooth_fall_alert(coords):
    """Sends a Bluetooth message with fall coordinates to the robot."""
    global client_sock
    
    if client_sock is None:
        print("No Bluetooth connection established")
        return False
    
    message = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "coordinates": {
            "x": coords[0],
            "y": coords[1]
        },
        "status": "FALL_DETECTED"
    }
    
    try:
        client_sock.send(json.dumps(message).encode('utf-8'))
        print(f"Bluetooth Alert Sent: Fall detected at coordinates {coords}")
        return True
    except Exception as e:
        print(f"Failed to send Bluetooth message: {e}")
        return False

def cleanup():
    """Clean up Bluetooth connections before exiting."""
    global client_sock, bt_server_sock, running
    
    running = False
    time.sleep(0.5)  # Give threads time to stop
    
    print("Cleaning up Bluetooth connections...")
    if client_sock:
        try:
            client_sock.close()
        except:
            pass
    
    if bt_server_sock:
        try:
            bt_server_sock.close()
        except:
            pass

# Set up the Bluetooth server
if not setup_bluetooth_server():
    print("Failed to set up Bluetooth server. Exiting.")
    exit(1)

# Start the server thread
server_thread = threading.Thread(target=bluetooth_server_thread)
server_thread.daemon = True
server_thread.start()

# Main loop
print("Starting fall detection and mapping...")
fall_detected = False
prev_fall_time = time.time() - 10  # Initialize to allow immediate detection

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Create a copy of the digital map for display
        display_map = digital_map.copy()
        
        # Process frame for pose detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(frame_rgb)
        
        # Draw pose landmarks on frame
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            
            # Check for fall
            is_fallen, coords = detect_fall(results.pose_landmarks.landmark, frame.shape[0])
            
            # Only trigger a new fall alert if enough time has passed since the last one
            current_time = time.time()
            if is_fallen and coords and (current_time - prev_fall_time) > 5:
                fall_detected = True
                prev_fall_time = current_time
                
                # Mark the fall position on the map
                cv2.circle(display_map, coords, 10, (0, 255, 0), -1)
                cv2.line(display_map, LAPTOP_POSITION, coords, (0, 0, 255), 2)
                
                # Send the fall position to the robot
                send_bluetooth_fall_alert(coords)
                
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
    cleanup()
    print("Resources released")
