import threading, time
from nlp_client import *

hi = EmerStop("nlp")
t = threading.Thread(target=hi.run, name="EmerStopFlask")
t.start()

speak("I'm ready to follow you")
print("I'm ready to follow you")
time.sleep(1.5)

while True:
    print("enter loop")
    hi.clear_status()
    time.sleep(1)
    print(hi.intent)
    if hi.intent == "follow_people":
        break

speak("I'm following you")
print("I'm following you")

while True:
    if hi.intent == "stop":
        speak("I'm stopping following you")
        break
    time.sleep(0.5)