
# speak to start
from nlp_client import *
import time

speak("I'm ready")
time.sleep(0.5)
speak("waiting for a customer to raise their hand")
speak("A customer raised their hand")
time.sleep(0.5)
# speak(desc)
# time.sleep(1.0)
speak("I'm coming")
speak("Can I get you something sir?")

while True:
    res = listen()
    if res.intent == "restaurant_order" and 'object' != '':
        break
    else:
        speak("Sorry I don't understand, Could you rephrase that?")

# listen
speak(f"Ok, I’m getting you a {res.object}")
print(res.object)

speak(f"here is your {res.object}")