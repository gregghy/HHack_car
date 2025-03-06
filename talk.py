import pyttsx3
engine = pyttsx3.init()

# For Mac, If you face error related to "pyobjc" when running the `init()` method :
# Install 9.0.1 version of pyobjc : "pip install pyobjc>=9.0.1"
text = "Should I call nine one one"

# RATE
rate = engine.getProperty('rate')   # getting details of current speaking rate
engine.setProperty('rate', 140)


# VOICE
voices = engine.getProperty('voices')       # getting details of current voice
#engine.setProperty('voice', voices[0].id)  # changing index, changes voices. o for male
engine.setProperty('voice', voices[1].id)

engine.say(text)
engine.runAndWait()

engine.stop()

