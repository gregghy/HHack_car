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

