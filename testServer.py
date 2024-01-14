#file made for testing llm_server.py
# DONOT USE
import requests
from ratfin import print
import json

def speak(text: str = "Hi my name is Walkie",
          log: bool = False) -> None:
    """Speak text using TTS server. Use sync = False to return immediately."""

    # try:
    if log:
        print("synthesizing...","blue")

    #* Requesting TTS server
    x = requests.post("http://localhost:5101/speak",
                    json={
                        'text': text,
                    })
    print(x.json())
    return x.json()

def listen(intent=True,
           log = True): # edited for testing
    if intent:
        # Prepare the payload
        payload = {"intent": True}
    else:
        # Prepare the payload
        payload = {"intent": False}

    # Post request to stt
    if log:
        print("listening...")
    response = requests.post("http://localhost:5101/listen", json=payload)
    if log:
        print("computing...")
    print(response.json())

def live_listen(intent=True,
           log = False):
    payload = {"intent": True}
    # Post request to stt
    response = requests.post("http://localhost:5101/live_listen", json=payload)


def get_intent(predicted_text, log=True):
    # Old function haven't been updated
    #* unlikely to be used
    response = {"recipient_id": "bot", "body": predicted_text}

    #TODO try and except UGLY.......
    #* Get intent
    r = requests.post(url="http://localhost:5101/get_intent",
                      json={
                          "sender": "bot",
                          "message": predicted_text
                      })
    rasa_json = r.json()[0]['text']
    rasa_json = json.loads(rasa_json)
    # print(rasa_json,"red")
    # print(response,"red")
    response.update(rasa_json)
    if log:
        print(f"\t{json.dumps(response, indent=4)}", "blue")
        print(f"\tlisten() sending back...", "green")
    return response

def ww_listen(text : str = "hey_walkie", log : bool = False) :
    """
    -Options-
    >>> ww_listen() # listens for "Hey Walkie"

    >>> ww_listen("hey_walkie") # listens for "Hey Walkie"
    >>> ww_listen("walkie_freeze") # listens for "Walkie Freeze"

    """
    if not isinstance(text, str):
            raise ValueError("Argument 'text' must be of type string")
    if log:
        print(f"posting to - http://localhost:5101/ww_listen/{text}","blue")

    response = requests.get(f"http://localhost:5101/ww_listen/{text}")
    return response.status_code == 200


def emerStop():
    return "hi"# not sure how to test yet

def testNoParam(testNumber,toTest,result):
    if(toTest()== result):
        print("test no."+testNumber+"passed")
        return
    print("test no."+testNumber+"failed")
    return


speak()
listen()
print(ww_listen())
# ww_listen()
# emerStop()