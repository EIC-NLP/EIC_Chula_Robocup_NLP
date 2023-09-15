# speak to start
from nlp_client import *
import time

# speak("I'm ready")
# time.sleep(0.5)
# speak("waiting for a customer to raise their hand")
# speak("A customer raised their hand")
# time.sleep(0.5)
# # speak(desc)
# # time.sleep(1.0)
# speak("I'm coming")
# speak("Can I get you something sir?")
while True:
    res_listen = listen()
    print(res_listen)
    if res_listen.intent == "restaurant_order" and 'object' != '':
        break
    else:
        speak("Sorry I don't understand, Could you rephrase that?")



# listen
speak(f"Ok, I’m getting you a {res_listen['object']}")
print(res_listen['object'])
speak(" I found a water bottle")
speak("here is your water")