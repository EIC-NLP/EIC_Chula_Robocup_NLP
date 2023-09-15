from nlp_client import live_listen
# This code is to test the live transcribe eature using speech recog


while True:
    input("enter to continue")
    x = live_listen(intent=True)
    print(f"{x}")
    # speak(x["body"])
    