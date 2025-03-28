import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import cv2
import RPi.GPIO as GPIO
import threading

# Digital Map Configuration
MAP_WIDTH = 500
MAP_HEIGHT = 400
# Robot's starting position (top-left corner)
ROBOT_POSITION = [50, 50]  # Using a list to allow modification inside callbacks
FALL_POSITION = None

# MQTT Configuration
BROKER_IP = "192.168.100.1"  # IP of laptop running the hotspot
BROKER_PORT = 1883
TOPIC_FALL_POSITION = "fall_detection/position"

# Initialize the map
digital_map = np.ones((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8) * 255
# Draw the starting positions
cv2.circle(digital_map, tuple(ROBOT_POSITION), 10, (255, 0, 0), -1)  # Robot in red
cv2.circle(digital_map, (450, 350), 10, (0, 0, 255), -1)  # Laptop in blue

# GPIO Setup
GPIO.setmode(GPIO.BCM)

# Motor pins setup - using the pins from your bot/drive.py
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

# Lock for thread safety when updating position
position_lock = threading.Lock()

# Flag to control navigation
navigating = False
arrived_at_destination = False

def get_distance():
    """Measure distance using HC-SR04 ultrasonic sensor."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if time.time() - start_time > 0.1:  # Timeout
            return 1000  # Return large value if timeout

    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
        if stop_time - start_time > 0.1:  # Timeout
            return 1000  # Return large value if timeout

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 343 m/s

    return distance

def move_robot(direction, duration=0.5):
    """
    Move the robot in the specified direction.
    If duration is 0, move continuously until stopped.
    """
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    
    if direction == "left":
        GPIO.output(front_left_motor_forward, GPIO.LOW)
        GPIO.output(front_left_motor_backward, GPIO.HIGH)
        GPIO.output(front_right_motor_forward, GPIO.HIGH)
        GPIO.output(front_right_motor_backward, GPIO.LOW)
    elif direction == "right":
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
    else:  # Stop
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)
    
    if duration > 0:
        time.sleep(duration)
        stop_robot()

def stop_robot():
    """Stop all motors."""
    for pin in motor_pins:
        GPIO.output(pin, GPIO.LOW)

def calculate_direction(current_pos, target_pos):
    """Calculate the direction to move based on current and target positions."""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    
    # Determine primary direction based on larger difference
    if abs(dx) > abs(dy):
        return "left" if dx < 0 else "right"
    else:
        return "forward" if dy < 0 else "backward"

def update_robot_position(direction):
    """Update the robot's position on the digital map."""
    with position_lock:
        if direction == "forward":
            ROBOT_POSITION[1] -= 5
        elif direction == "backward":
            ROBOT_POSITION[1] += 5
        elif direction == "left":
            ROBOT_POSITION[0] -= 5
        elif direction == "right":
            ROBOT_POSITION[0] += 5
        
        # Ensure robot stays within map bounds
        ROBOT_POSITION[0] = max(0, min(MAP_WIDTH-1, ROBOT_POSITION[0]))
        ROBOT_POSITION[1] = max(0, min(MAP_HEIGHT-1, ROBOT_POSITION[1]))

def navigate_to_fall():
    """Navigate the robot to the fall position."""
    global navigating, arrived_at_destination, FALL_POSITION
    
    if FALL_POSITION is None:
        print("No fall position set")
        return
    
    navigating = True
    arrived_at_destination = False
    
    print(f"Starting navigation to fall position: {FALL_POSITION}")
    
    try:
        while navigating and not arrived_at_destination:
            # Check for obstacles
            distance = get_distance()
            if distance < 30:  # Less than 30cm is an obstacle
                print(f"Obstacle detected at {distance}cm. Stopping.")
                stop_robot()
                time.sleep(1)
                continue
            
            # Calculate direction to target
            with position_lock:
                current_pos = ROBOT_POSITION.copy()
            
            # Calculate distance to target
            dx = FALL_POSITION[0] - current_pos[0]
            dy = FALL_POSITION[1] - current_pos[1]
            distance_to_target = np.sqrt(dx**2 + dy**2)
            
            if distance_to_target < 20:  # Close enough to target
                print("Arrived at fall position")
                stop_robot()
                arrived_at_destination = True
                break
            
            # Determine direction of movement
            direction = calculate_direction(current_pos, FALL_POSITION)
            
            # Move robot in calculated direction
            print(f"Moving {direction} towards target. Distance: {distance_to_target:.1f}")
            move_robot(direction, 0.5)
            
            # Update position based on movement
            update_robot_position(direction)
            
            # Update map for visualization (if display is connected)
            draw_map()
            
            time.sleep(0.1)  # Small delay for control loop
    
    except Exception as e:
        print(f"Navigation error: {e}")
    finally:
        stop_robot()
        navigating = False

def draw_map():
    """Draw the current map with robot and fall positions."""
    map_display = np.ones((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8) * 255
    
    # Draw laptop position
    cv2.circle(map_display, (450, 350), 10, (0, 0, 255), -1)
    
    # Draw robot position
    with position_lock:
        cv2.circle(map_display, tuple(ROBOT_POSITION), 10, (255, 0, 0), -1)
    
    # Draw fall position if set
    if FALL_POSITION is not None:
        cv2.circle(map_display, FALL_POSITION, 10, (0, 255, 0), -1)
        # Draw line from robot to fall position
        with position_lock:
            cv2.line(map_display, tuple(ROBOT_POSITION), FALL_POSITION, (255, 0, 255), 2)
    
    # Display the map
    cv2.imshow("Robot Navigation Map", map_display)
    cv2.waitKey(1)

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe(TOPIC_FALL_POSITION)
    print(f"Subscribed to {TOPIC_FALL_POSITION}")

def on_message(client, userdata, msg):
    global FALL_POSITION, navigating
    
    try:
        data = json.loads(msg.payload.decode())
        print(f"Received message: {data}")
        
        if "coordinates" in data and data.get("status") == "FALL_DETECTED":
            coords = data["coordinates"]
            FALL_POSITION = (coords["x"], coords["y"])
            print(f"Fall position set to: {FALL_POSITION}")
            
            # Start navigation in a separate thread
            if not navigating:
                nav_thread = threading.Thread(target=navigate_to_fall)
                nav_thread.daemon = True
                nav_thread.start()
    
    except json.JSONDecodeError:
        print(f"Error decoding JSON message: {msg.payload.decode()}")
    except Exception as e:
        print(f"Error processing message: {e}")

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to MQTT broker
try:
    print(f"Connecting to MQTT broker at {BROKER_IP}:{BROKER_PORT}...")
    client.connect(BROKER_IP, BROKER_PORT, 60)
    client.loop_start()
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")

try:
    print("Robot navigation system started")
    print("Waiting for fall detection alerts...")
    
    # Main loop - keep program running and update map display
    while True:
        draw_map()
        time.sleep(0.1)
        
        # Check for keyboard input to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("Program interrupted by user")
finally:
    # Clean up
    client.loop_stop()
    client.disconnect()
    stop_robot()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("Resources released")
