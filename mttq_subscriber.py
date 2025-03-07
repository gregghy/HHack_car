import paho.mqtt.client as mqtt

# MQTT Broker Configuration
BROKER_IP = "localhost"  # or the IP address of the MQTT broker Raspberry Pi
BROKER_PORT = 1883
TOPIC = "fall_detection/alert"

def on_message(client, userdata, message):
    """Callback function to handle incoming messages."""
    print(f"Alert Received: {message.payload.decode()}")

# Initialize MQTT Client
client = mqtt.Client()

# Set callback function for message arrival
client.on_message = on_message

# Connect to the broker
client.connect(BROKER_IP, BROKER_PORT, 60)

# Subscribe to the topic
client.subscribe(TOPIC)

# Loop forever to listen for messages
client.loop_forever()
