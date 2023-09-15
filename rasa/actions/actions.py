# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions

# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List
from sqlalchemy import true

from urllib3 import Retry

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet, SessionStarted, ActionExecuted, EventType
import requests
import json
import yaml
import ast

""" CONFIG """

VERBOSE = False
# VERBOSE = False
""" END CONFIG """

ros_url = "http://localhost:5000/ros"
print("ROS URL : {}".format(ros_url))
object_list = []
try:
    with open("/home/eic/nlp/rasa-nlu-v1/data/lookups/object_list.txt",
              "r") as f:
        lines = f.readlines()
        for line in lines:
            object_list.append(line.strip())
except:
    pass


def actionVerbose(tracker):
    if VERBOSE:
        tracker_latest_message = tracker.latest_message
        print('\033[0;36m' + "\ntracker.latest_message=" + '\033[0m' +
              f"\n{json.dumps(tracker_latest_message,indent=2)}")
        #saves tracker_latest_message to file
        with open("tracker_latest_message.json", "w") as f:
            f.write(json.dumps(tracker_latest_message, indent=4))

        # tracker_dict = tracker.__dict__
        # print('\033[0;36m' + "\ntracker.__dict__=" + '\033[0m' +
        #       f"\n{json.dumps(tracker_dict,indent=2)}")
        # #saves tracker_dict to file
        # with open("tracker_dict.json", "w") as f:
        #     f.write(json.dumps(tracker_dict, indent=4))


def define_object(object: str) -> bool:
    for x in object_list:
        if x in object:
            return True
    return False


def property_object(object: str) -> list:
    property_ob = list()
    with open('property_object.yml') as file:
        document = yaml.full_load(file)
        for name, member in document.items():
            if object in member:
                property_ob.append(name)
    return property_ob


def speak(text):
    try:
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text': text})
        return x
    except:
        print("error to connect speak api.")


class ActionHelloWorld(Action):

    def name(self) -> Text:
        return "action_hello_world"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        actionVerbose(tracker)
        dispatcher.utter_message(text="Hello World!")
        print("hello world")
        with open("testtest.txt", "w") as f:
            f.write("Hello World!")

        return []


class ActionSendTask(Action):
    # Examples
    """ 
    - rule: stop
    steps:
    - intent: stop
    - action: action_send_task 

    this activates action_send_task 
    """

    def name(self) -> Text:
        return "action_send_task"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        data = {  # return to ros
            "intent": tracker.latest_message['intent']['name'],
            "confidence": tracker.latest_message['intent']['confidence'],
            "text": tracker.latest_message.get('text')
        }

        # actionVerbose(tracker)
        # print(tracker.latest_message)


        # tracker.get_latest_entity_values(entity_type=”city”, entity_role=”destination”) 
        # #use ast to convert tracker.latest_message['entities'] to dict
        # tracker = ast.literal_eval(tracker.latest_message)
        # get all entities
        for x in tracker.latest_message['entities']:
            #convert x to dict stored in temp
            temp = ast.literal_eval(json.dumps(x))
            if not "vb" in temp['entity']:
                
                # if entity has role, add role after entity name
                if temp.get('role') != None:
                    data[temp['entity'] + "_" + temp['role']] = temp['value']
                else:
                    data[temp['entity']] = temp['value']

        # label object
        if data.get('object') != None and define_object(
                data['object']) and data['intent'] == "find_people":
            data['intent'] = "find_object"
            data.pop('people', None)
        elif data['intent'] == "find_people":
            if data.get('people') == None and data.get('object'):
                data['people'] = data['object']
            data.pop('object', None)


        object1 = json.dumps(data)
        print(object1)
        print()
        # post to ROS
        try:
            x = requests.post(ros_url, json=object1)
        except:
            # print("cannot connect ros")
            pass
        dispatcher.utter_message(text=object1)
        return []


# class ActionFollowPeople(Action):
#     def name(self):
#         return "action_follow_people"

#     def run(self, dispatcher, tracker, domain):
#         data = {
#             "intent": tracker.latest_message['intent'].get("name")
#         }
#         # get all entities
#         for x in tracker.latest_message['entities'] :
#             if x.get('entity') == "people":
#                 data['people'] = x.get('value')

#         try :
#             people = data['people']
#             if people == 'me':
#                 people = 'you'
#             response = f"I will follow {people}"
#             object1 =json.dumps(data)
#             dispatcher.utter_message(text=object1)
#             print(object1)
#             dispatcher.utter_message(text=response)
#             speak(response)
#         except:
#             pass

#         #debug
#         print(data)
#         return []


class ActionWhatIsThat(Action):

    def name(self) -> Text:
        return "action_what_is_that"

    def run(self, dispatcher, tracker, domain):
        data = {"intent": tracker.latest_message['intent'].get("name")}
        actionVerbose(tracker)
        # get all entities
        for x in tracker.latest_message['entities']:
            if x.get('entity') == "demonstrative":
                data['demonstrative'] = x.get('value')

        object1 = json.dumps(data)
        print(object1)
        response = f"debug: what is {data['demonstrative']}"
        try:
            x = requests.post(ros_url, json=object1)
        except:
            print("cannot connect ros")

        dispatcher.utter_message(text=object1)
        dispatcher.utter_message(text=response)
        # speak(response)

        return []


class ActionTellWeather(Action):

    def name(self) -> Text:
        return "action_tell_weather"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        try:
            actionVerbose(tracker)
            actionVerbose(tracker)
            ret = requests.get(
                'http://www.7timer.info/bin/api.pl?lon=100.5352&lat=13.7401&product=civil&output=json'
            )
            data = ret.json()
            weather = data["dataseries"][0]["weather"]
            dispatcher.utter_message(text="The weather is " + weather)
            speak("The weather is " + weather)
        except:
            dispatcher.utter_message(
                text="An error occurred retrieving the weather information..")
        return []


# class ActionReady(Action):

#     def name(self) -> Text:
#         return "action_ready"

#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker,
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

#         try:
#             actionVerbose(tracker)
#             response = f'Hi, my name is walkie. I am from chulalongkorn university. I am very ready to the competition. nice to meet you all and fighting team'
#             speak(response)
#         except:
#             pass
#         return []
