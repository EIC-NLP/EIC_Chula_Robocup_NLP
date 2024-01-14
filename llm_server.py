#server to handle all the same requests as src_client_pkg, using this server as a webhosting server to seperate the database from the running code. 
# TODO implement ww_listen and emerstop support
# implement llm module
# DUE ?


from flask import Flask, request, jsonify
from config import *   
import json
import requests
import os
from datetime import datetime
from scipy.io import wavfile
import io
import time
from src_client_pkg.nlp_client.response import Response

class LLMServer:#server class
    def __init__(self):
        self.app = Flask("LLMServer")
        self.app.add_url_rule("/", "home", self.hello_world, methods=["POST","GET"])#set route to handle requests
        self.app.add_url_rule("/speak","speak",self.handleSpeak,methods=["POST"])#handles for each function
        self.app.add_url_rule("/listen","listen",self.handleListen,methods=["POST"])
        self.app.add_url_rule("/get_intent","get_intent",self.handleGetIntent,methods=["POST"])
        self.app.add_url_rule("/live_listen","live_listen",self.handleLiveListen,methods=["POST"])
        self.app.add_url_rule("/ww_listen/<text>","ww_listen",self.handleWWListen,methods=["GET"])
        self.app.add_url_rule("/EmerStop","EmerStop",self.handleEmerStop,methods=["GET"])
    
    

    def hello_world(self):#home method
        print(request.method)
        return "<p>Hello, World!</p>"
    
    def handleSpeak(self):#methods to handle different requests, not implemented fully yet
        if request.method == 'POST':
            cache = json.loads(request.data)
            print(cache)
            return jsonify(cache)

    def handleListen(self):
       if request.method == 'POST':
            cache = json.loads(request.data)
            print(cache)
            return jsonify(cache)
        
    def handleLiveListen(self):
        if request.method == 'POST':
            cache = json.loads(request.data)
            print(cache)
            return jsonify(cache)
    
    def handleGetIntent(self):
        if request.method == 'POST':
            cache = json.loads(request.data)
            print(cache)
            return jsonify(cache)

    def handleWWListen(self,text):
        if request.method == 'GET':
            if text == "hey_walkie":
                return "i am walkie"
            
    
    def handleEmerStop(self):
        return  #should return a .json


    def run(self):#code to run flask server
        try:
            self.app.run(host="0.0.0.0",
                        port=int(stt_url.split(":")[-1][:-1:]),
                        threaded=True,
                        debug=True)
            print("\033[0;35m" + f"\nlisten(GET): {stt_url}" +"\n\033[0m")
        except OSError:
            print("bruh moment")
            exit()

    # def run(self)
    #     try:
    #         print("hi")
    #         self.app.run(host="0.0.0.0")
    #     except OSError:
    #         #print(colored(("Port already in use, please change port in .env", "red")
    #         print("bruhMoment")
    #         exit()

if __name__ == "__main__":# instatiate and run the server
    server = LLMServer()
    server.run()
    print("server stopped")
