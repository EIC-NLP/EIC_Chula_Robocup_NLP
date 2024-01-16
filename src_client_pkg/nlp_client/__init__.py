

__version__ = '1.14.0'
"""
Date: 28 Jan 2023
"""
import requests
import json
import ast
from termcolor import colored
# for ChatGPT online Function
import openai

## In house variable

import json
# from response import Response
class Response:
    """Response for the NLP-pipeline.
    Possible Entities: [object, furniture, storage, adj_object, people, people_action, place_object, position, demonstrative, rpos, door_action, room]

    Text from stt
    Intent from NLU Rasa

    Example of how to use Response for NLP Robocup ::

        >>> x = Response()
        >>> print(x)
        Response(
            text=''
            intent=''
            confidence=0.0
            object=''
            people=''
        )
        >>> x.join_json("{
            \"intent\": \"restaurant_order\",
            \"object\": \"coca-cola\",
            \"people\": \"me\",
            \"place\": \"your mum\"
            }")
        >>> print(x)
        Response(
            text=''
            intent='restaurant_order'
            confidence=0.0
            object='coca-cola'
            people='me'
            place='your mum'
        )
        >>> x.your_mum = "fat"
        >>> print(x)
        Response(
            text=''
            intent='restaurant_order'
            confidence=0.0
            object='coca-cola'
            people='me'
            place='your mum'
            your_mum = "fat" <--
        )

    """

    def __init__(
        self,
        text: str = "",
        intent: str = "",
        confidence: float = 0.00,
        object: str = "",
        people: str = "",
        room: str = "",

        **kwargs  # To handle future attributes
    ):
        self.text = text
        self.intent = intent
        self.confidence = confidence
        self.object = object
        self.people = people
        self.room = room

        # Store any additional attributes as instance variables
        for key, value in kwargs.items():
            setattr(self, key, value)

    def join_json(self, text: str, verbose = False):
        json_dict = json.loads(str(text))
        print(f"{json_dict=}")
        for key, value in json_dict.items():
            setattr(self, key, value)

    def join_dict(self, in_dict=None, verbose = False):
        # print(in_dict)
        for key, value in in_dict.items():
            setattr(self, key, value)

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def __str__(self):
        attributes = [
            f'{attr}={getattr(self, attr)!r}'
            for attr in vars(self)
            if not callable(getattr(self, attr)) and not attr.startswith("__")
        ]
        output = '\n\t'.join(attributes)
        return f'''Response(\n\t{output}\n)'''

def speak(text: str = "Hi my name is Walkie",
          log: bool = False) -> None:
    """Speak text using TTS server. Use sync = False to return immediately."""

    # try:
    if log:
        print(colored('synthesizing', 'blue'))

    #* Requesting TTS server
    x = requests.post("http://localhost:5003/tts",
                    json={
                        'text': text,
                    })
    print(x.json())
    return x.json()

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
        print(f"posting to - http://localhost:5100/{text}","blue")

    response = requests.get(f"http://localhost:5100/{text}")
    return response.status_code == 200


def listen(intent=True,
           log = False) -> Response:  # go to stt server, By-pass wakeword
    """
    >>> listen()
    Response(
        text='my favourite drink is Coca-Cola'
        intent='favorite_object'
        confidence=0.89
        object='Coca-Cola'
        people=''
    )

    >>> listen(intent=False)
    Response(
        text='my favourite drink is Coca-Cola'
        intent=''
        confidence=0.0
        object=''
        people=''
    )
    """
    if intent:
        # Prepare the payload
        payload = {"intent": True}
    else:
        # Prepare the payload
        payload = {"intent": False}


    # Post request to stt
    if log:
        print("listening...")
    response = requests.post("http://localhost:5101/", json=payload)
    if log:
        print("computing...")

    if intent:

            rasa_res = response.json()
            if log:
                print(repr(rasa_res))
                # {'body': '{"intent": "restaurant_order", "confidence": 0.9962742328643799, "text": "Can I have a Coca-Cola please?", "object": "Coca-Cola"}'}

            rasa_dict = ast.literal_eval(str(rasa_res))

            if log:
                print(repr(rasa_dict))
                # {'intent': 'restaurant_order', 'confidence': 0.9962742328643799, 'text': 'Can I have a Coca-Cola please?', 'object': 'Coca-Cola'}

            # Create a Response object
            obj = Response()

            # Join the dictionary to the Response object
            obj.join_dict(rasa_dict)
            """ Response(
                        text='Can I have a Coca-Cola please?'
                        intent='restaurant_order'
                        confidence=0.9962742328643799
                        object='Coca-Cola'
                        people=''
                ) """

            # Return the Response object
            return obj
    else:
        # Your received response
        res_txt = response.json()
        # {'body': "{'text': ' How are you today?'}"}

        if log:
            print(repr(res_txt))

        # Parse the 'body' string into a dictionary
        body_dict = ast.literal_eval(str(res_txt['body']))
        # {'text': ' How are you today?'}

        if log:
            print(repr(body_dict))

        # Extract 'text' from the 'body' dictionary
        text = body_dict['text']
        # "How are you today?"

        if log:
            print(repr(text))

        # Create a Response object with the extracted text
        obj = Response(text=text)
        """ Response(
                    text=' How are you today?'
                    intent=''
                    confidence=0.0
                    object=''
                    people=''
        ) """


        # return a Response object
        return obj


def live_listen(intent=True,
           log = False):  # go to stt server, By-pass wakeword

    if intent:
        # Prepare the payload
        payload = {"intent": True}
    else:
        # Prepare the payload
        payload = {"intent": False}

    # Post request to stt
    response = requests.post("http://localhost:5101/live_listen", json=payload)

    if intent:
            rasa_res = response.json()

            if log:
                print(rasa_res)
                # {'body': '{"intent": "restaurant_order", "confidence": 0.9962742328643799, "text": "Can I have a Coca-Cola please?", "object": "Coca-Cola"}'}

            rasa_dict = ast.literal_eval(rasa_res['body'])

            if log:
                print(rasa_dict)
                # {'intent': 'restaurant_order', 'confidence': 0.9962742328643799, 'text': 'Can I have a Coca-Cola please?', 'object': 'Coca-Cola'}

            # Create a Response object
            obj = Response()

            # Join the dictionary to the Response object
            obj.join_dict(rasa_dict)
            """ Response(
                        text='Can I have a Coca-Cola please?'
                        intent='restaurant_order'
                        confidence=0.9962742328643799
                        object='Coca-Cola'
                        people=''
                ) """

            # Return the Response object
            return obj
    else:
        # Your received response
        res_txt = response.json()
        # {'body': "{'text': ' How are you today?'}"}

        # Parse the 'body' string into a dictionary
        body_dict = ast.literal_eval(res_txt['body'])
        # {'text': ' How are you today?'}

        # Extract 'text' from the 'body' dictionary
        text = body_dict['text']
        # "How are you today?"

        # Create a Response object with the extracted text
        obj = Response(text=text)
        """ Response(
                    text=' How are you today?'
                    intent=''
                    confidence=0.0
                    object=''
                    people=''
        ) """


        # return a Response object
        return obj



def get_intent(predicted_text, log=True):
    # Old function haven't been updated
    #* unlikely to be used
    response = {"recipient_id": "bot", "body": predicted_text}

    #TODO try and except UGLY.......
    #* Get intent
    r = requests.post(url="http://localhost:5005/webhooks/rest/webhook",
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


messages = [{
            "role": "system",
            "content" : """You’re a kind helpful assistant robot,
                            respond back to me what my commands were but rephrase it like a assistant would
                            by accepting my request. don't ask a question back just do as I says. For example,
                            if I ask you to retrieve a coke. You should respond with something like "Certainly, grabing you a coke now" but make the sentence dynamic dont actually use the word certainly its too formal.
                            This is a role-play"""}]

def query_llm(prompt='', log=False, clear=False) -> str:
        global messages

    #* Initialisation
        if clear:
            messages = [{
            "role": "system",
            "content" : """You’re a kind helpful assistant robot,
                            respond back to me what my commands were but rephrase it like a assistant would
                            by accepting my request. don't ask a question back just do as I says. For example,
                            if I ask you to retrieve a coke. You should respond with something like "Certainly, grabing you a coke now" but make the sentence dynamic dont actually use the word certainly its too formal.
                            This is a role-play"""}]
        else:
            # init the key
            openai.api_key = gpt_key

            # Copy the assistant mode prompt
            messages = messages.copy()

            # Log the execution stage
            if log:
                print(prompt,'blue')


            # Append to messages
            messages.append({"role": "user", "content": prompt})

            # packaged the messages and post request
            completion = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages
            )
            # extract the content of the post request
            chat_response = completion.choices[0].message.content

            # Log the execution stage
            print(f'(ChatGPTQuery): ChatGPT response: {chat_response}','blue')

            # Save the response to userdata
            return chat_response


class EmerStop():
    # definitely not working

    def __init__(self, name):

        self.name = name
        self.confidence = None
        self.intent = None


    def run(self):
        while True:
            try:
                x = requests.get("http://localhost:5101/").json()
                if "intent" in x:
                    # print("in")
                    if x["intent"] == "stop" and x["confidence"] > 0.62:
                        print("STOPPINGGGG........", "red")
                        self.confidence = x["confidence"]
                        self.intent = x["intent"]
            except:
                pass

    def clear_status(self):
        self.confidence = None
        self.intent = None


def main():
    # os.system("clear")
    # import threading, time
    # hi = EmerStop("nlp")
    # t = threading.Thread(target=hi.run, name="EmerStopFlask")
    # t.start()
    # while True:
    #     time.sleep(4)
    #     print(hi.intent, hi.confidence)

    #check

    # for i in range(10):
    #     speak(
    #         text = "WAKE THE FUCK UP",
    #         voice = "en-US-JaneNeural" ,
    #         style = "shouting" # can be shouting, normal

    #     )
    # print("this is speak")
    # print(speak("say stop motherfucker"))
    # print(speak("say "))
    # print(listen())
    # while True:
    # x = dict(ww_listen())
    # print(ww_listen())
    # print(json.dumps(listen(), indent=4))
    # print(json.dumps(ww_listen(), indent=4))
    # if x['intent'] == "stop":
    #     print("STOPPINGGGG........","red")
    #     speak("stop")
    pass


main()
# print(ww_listen())
# print(speak("hi there this is a test for speak"))