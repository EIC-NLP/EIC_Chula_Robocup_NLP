from nlp_client import listen


while True:
    input("enter to continue, stt")
    print("Speak now")
    x = listen(intent=False, log=False)
    print(f"{x}")
    # write to a file
    with open("test.txt", "a") as f:
        f.write(f"{x.text}\n")

    # speak(x.text, online=False)
    # speak(x["body"])
