import requests
from ratfin import *
from nlp_client import ww_listen, listen, speak
import json

""" 
Funcitons:
    speak()
    listen()
    ww_listen() 
"""
""" intent: restaurant_order """

def main():
    while True:
        result = listen()

        print(result)
        input()
        clearterm() 


    # print("this is speak")
    # print(speak("say stop motherfucker"))
    # print(speak("Hi there my name is Walkie, I will be your personal assistant"))
    # result = ww_listen()s
    # print("this is the intent from the response from ww_listen(): " + result["intent"])
    # speak(result["body"])
    # while True:
        # x = dict(ww_listen())
    # print(ww_listen())
    # print(json.dumps(listen(), indent=4))
    # print(json.dumps(listen(), indent=4))
        # if x['intent'] == "stop":
        #     printclr("STOPPINGGGG........","red")
        #     speak("stop")
    # my_list = dict({
    #     "intent": "restaurant_order",
    #     "object": "pizza"
    # })

main()
    # print(ww_listen())
# print(speak("hi there this is a test for speak"))
