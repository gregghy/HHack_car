import pyttsx3
import time

# For Mac, If you face error related to "pyobjc" when running the `init()` method :
# Install 9.0.1 version of pyobjc : "pip install pyobjc>=9.0.1"
#text = "You are on the ground."
text2 = "Do I call nine one one?"

# RATE
def speak(text):
    engine = pyttsx3.init()
    engine.setProperty('rate', 160)
    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[1].id)  # Adjust if needed
    #engine.say(text)
    #engine.runAndWait()
    engine.say(text)
    engine.runAndWait()

speak(text2)

engine.stop()
