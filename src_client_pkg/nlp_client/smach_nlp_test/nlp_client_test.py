import requests
from ratfin import *
from nlp_client import *
import json

def main():
    clearterm() 

    # print("this is speak")
    # print(speak("say stop motherfucker"))
    print(speak("say "))
    # print(listen())
    # while True:
        # x = dict(ww_listen())
    # print(ww_listen())
    # print(json.dumps(listen(), indent=4))
    print(json.dumps(listen(), indent=4))
        # if x['intent'] == "stop":
        #     printclr("STOPPINGGGG........","red")
        #     speak("stop")

main()
    # print(ww_listen())
# print(speak("hi there this is a test for speak"))
