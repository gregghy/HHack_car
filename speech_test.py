import os
import sounddevice as sd
import vosk
import sys
import json
import numpy as np
import threading

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
running = True

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
    global running
    if "stop" in text.lower():
        print("\n🛑 Stopping...")
        running = False
        sys.exit(0)
    elif "hello" in text.lower():
        print("\n👋 Hello! How can I help?")

# Threaded function to run speech recognition
def start_recognition():
    global running
    print("\n🎤 Listening... (Say 'stop' to exit)")
    
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype="int16",
                        callback=callback, blocksize=BUFFER_SIZE):
        while running:
            pass  # Keep alive without blocking

# Run recognition in a high-priority thread
recognition_thread = threading.Thread(target=start_recognition, daemon=True)
recognition_thread.start()

# Keep script running
while running:
    pass

