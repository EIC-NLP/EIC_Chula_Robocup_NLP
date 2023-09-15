from nlp_client import *


while True:
    input("enter to continue, stt")
    x = listen(intent=False, log=False)
    print(f"{x}")
    speak(x.text, online=True)
    # speak(x["body"])
     