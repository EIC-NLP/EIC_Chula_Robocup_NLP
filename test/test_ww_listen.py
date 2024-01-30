from nlp_client import *

# wakeword ==> "hey walkie"
while True:
    x = ww_listen()

    print(x)

    # y = listen(intent=True, log=False)
    # print(y)
    # # speak(x["body"])
    # input()
