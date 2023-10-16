from nlp_client import listen


while True:
    input("enter to continue, stt")
    x = listen(intent=False, log=False)
    print(f"{x}")
    # speak(x.text, online=False)
    # speak(x["body"])
