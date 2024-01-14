from nlp_client import *
from termcolor import colored

# print(colored("testing normal stt with intent","yellow")
x = listen(intent=True, log=True)
print(colored(x.text,'blue'))

# print(colored("testing normal stt without intent","yellow")
x = listen(intent=False, log=True)
print(colored(x.text,'blue'))

# print(colored("testing TTS Azure","yellow")
# speak("Hello there my name is Game",online=True)

# print(colored("testing TTS Mimic","yellow")
# speak("Hello there. my name is Game",online=False)
