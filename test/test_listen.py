from termcolor import colored
from nlp_client import listen


while True:
    input("enter to continue, stt")
    print("Speak now")
    x = listen(intent=True, log=False)
    print(f"{x}")
    print(x.text)
    print(x.confidence)
    print(x.people if x.people else "no people")

    # print(x.room_from)
    # print(colored(x.room_from, 'red'))
    # # write to a file
    # with open("test.txt", "a") as f:
    #     f.write(f"{x.text}\n")

    # speak(x.text, online=False)
    # speak(x["body"])