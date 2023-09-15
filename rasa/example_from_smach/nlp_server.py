#!/usr/bin/env python

"""
sudo netstat -nlp | grep 5000
kill -9 <pid_id>
"""

from flask import Flask, request
import threading
import time
import json
import requests

class SpeechToText():

    def __init__(self, name):
        self.app = Flask(name)
        self.app.add_url_rule("/", "index", self.greet)
        self.app.add_url_rule("/ros", "intent", self.intent, methods=["POST"])
        self.body = None

    def run(self):
        self.app.run(host = "0.0.0.0", port = 5000, threaded = True)

    def greet(self):
        return "<h1>Hello World</h1>"

    def intent(self):
        body = request.get_json()
        self.body = json.loads(body)
        print(self.body)
        return {"success" : True}
    
    def clear(self):
        self.body = None

    def listen(self):
        requests.get("http://0.0.0.0:5001/stt")

def speak(text) :
    try :
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text':text}) # post the string to the tts_server
        return x
    except :
        print("error to connect speak api.")

# class Text_to_speech(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['continue_succeeded'],
#                                     input_keys=['tts_input'])
#     def execute(self, userdata):
#         rospy.loginfo('Executing state Text_to_speech')
#         print("You are pointing at " + userdata.tts_input)
#         if userdata.tts_input == 'no_object':
#             # text to Speech
#             d = {"text" : "You are not pointing at any object"}
#             requests.post('http://localhost:5003/tts', json=d)
#         else:
#             # text to speech
#             d = {"text" : "You are pointing at " + userdata.tts_input}
#             requests.post('http://localhost:5003/tts', json=d)
#         return 'continue_succeeded'

if __name__=="__main__":
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()
    while True:
        print(stt.body)
        time.sleep(0.1)
    