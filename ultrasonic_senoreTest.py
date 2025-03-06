import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for TRIG and ECHO
TRIG = 6  # GPIO pin 6 for TRIG
ECHO = 5  # GPIO pin 5 for ECHO

# Set up the trigger and echo pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Function to measure the distance
def measure_distance():
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

try:
    while True:
        distance = measure_distance()
        if distance != -1:  # Only print the distance if it's valid
            print(f"Distance: {distance} cm")
        else:
            print("Measurement failed, retrying...")
        time.sleep(0.5)  # Wait for 500ms (0.5 seconds) before measuring again
except KeyboardInterrupt:
    print("Measurement stopped by User")
finally:
    GPIO.cleanup()  # Clean up GPIO settings on exit
