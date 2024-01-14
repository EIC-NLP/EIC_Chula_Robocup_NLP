import ast
import os
import yaml
import requests
from termcolor import colored
import re

global counter, sub_counter

log = False
counter = 0
sub_counter = 0
import time
timing = True
global numcorrect, numwrong

numcorrect = 0
numwrong = 0

list_band_names = [
    "nlu.yml",
    # "nlu_general_smach_actions.yml",
    # "nlu_specific_task.yml",
    # "nlu_gpsr.yml",
    "stories.yml",
    "rules.yml",
    "people.yml",
    "verb.yml",
    "placement.yml",
    "object.yml",


]
file_read = []


def process_files(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.yaml') or file.endswith('.yml'):
                process_file(os.path.join(root, file))

def process_file(file_path):
    global counter, sub_counter
    filename = file_path.split('/')[-1]  # get the last part of the path

    if filename in list_band_names:
        print(colored(f"skipping {filename}","yellow"))
    else:
        file_read.append(filename)
        with open(file_path, 'r') as file:
            print(colored(f"\n\nreading from: {file_path}","yellow"))
            lines = file.readlines()
            if len(lines) > 1 and 'nlu' in lines[1]:
                file.seek(0)
                yaml_content = yaml.safe_load(file)
                if 'nlu' in yaml_content:
                    for intent_block in yaml_content['nlu']:
                        if 'intent' in intent_block and 'examples' in intent_block and intent_block['examples'] is not None:
                            intent = intent_block['intent']
                            if '/' in intent:
                                print(f"Skipping intent '{intent}' due to '/' character")
                                continue
                            examples = intent_block['examples'].split('\n')
                            print(colored(f"\n[{counter}] Testing on intent: {intent}","blue"))
                            for example in examples:
                                if example.strip() != '':
                                    sub_counter += 1
                                    process_example(intent, example)

                            sub_counter = 0
                    counter += 1


def process_example(intent, example):
    cleaned_example = re.sub(r'\[|\]|\(.*?\)|\{.*?\}', '', example)
    cleaned_example = cleaned_example.strip()
    post_request(intent, cleaned_example)
text = "get the milk in the kitchen and go to the living room"
def post_request(intent, message, force_green=False):
    global counter, sub_counter
    link = "http://localhost:5005/webhooks/rest/webhook"  # replace with your actual API url

    if timing: start = time.time()
    if log:print(f"Sending: {message}") ;print()
    response = requests.post(url=link, json={"sender": "bot", "message": message})
    # response = requests.post(url=link, json={"sender": "bot", "message": message[2::]})
    if timing: out_time = str(time.time()-start)[:5] + "s"

    global numcorrect, numwrong

    if response.status_code == 200:
        json_response = response.json()
        try:
            if log:print(json_response)
            json_response = json_response[0]
            if log:print(json_response)

            dict_response = ast.literal_eval(json_response["text"])
            response_intent = dict_response["intent"]
            if response_intent == intent or force_green:
                print(colored(f"[{out_time}] [{counter}.{sub_counter}]"+str(json_response["text"]),"green"))
                numcorrect += 1
            else:
                print(colored(f"[{out_time}] [{counter}.{sub_counter}]"+str(json_response["text"]),"red"))
                numwrong += 1
        except:
            print(colored(f"[{out_time}] [{counter}.{sub_counter}]"+str(json_response),"red"))
            numwrong += 1

    else:
        print(colored(f"[{out_time}] [{counter}.{sub_counter}]"+str(json_response),"red"))
        numwrong += 1
    return response_intent

if __name__ == '__main__':
    CUSTOM_DATA = False

    custom_request = """
    hey walkie come over here
    hello there
    say hello to me
    where you are standing is th origin

    """.strip().split('\n')
    if CUSTOM_DATA:
        for i in custom_request:
            post_request(intent="",message=i, force_green=True)


    start_program = time.time()
    directory = 'rasa/data'  # replace with the path to your directory
    process_files(directory)
    print(f"File read: ")
    for i in file_read:
        print(i)

    print(colored(f"-------------Total correct: {numcorrect}","green"))
    print(colored(f"-------------Total wrong: {numwrong}","red"))


    print(colored(f"-------------Total time taken: {str(time.time()-start_program)[:8]}s","yellow"))
    # print in minutes
    print(colored(f"-------------Total time taken: {str((time.time()-start_program)/60)[:6]}m","yellow"))


    # DUMB TESTER
    #     longmf = """
    # Meet William at the sink and follow him
    # Meet Charlie at the bed and follow him to the bedroom
    # Please give me the left most object from the cupboard
    # Tell me how many people in the dining room are boys
    # Robot please meet Charlie at the desk and follow her to the corridor
    # Please escort Patricia from the bed to the bookcase
    # Give the spoon to me
    # Follow Robin
    # Navigate to the couch, meet Alex, and take him to the entrance
    # Please meet Robert at the desk and follow him
    # Meet Jennifer at the end table, follow her, and escort her back
    # Please face Francis at the sink and take her to her taxi
    # Follow Patricia from the couch to the bedroom
    # Please tell the day of the week to the person pointing to the right in the living room
    # Meet Robin at the sink, follow him, and go to the kitchen
    # Could you place the bowl on the end table
    # Could you go to the bed, meet Robin, and accompany her to the end table
    # Take out the debris
    # Take the scrubby to the cupboard
    # Follow Linda from the sink to the living room

    # Could you Tell me the name of the person in the bedroom
    # Could you meet Elizabeth at the bookcase, follow her, and accompany her back
    # Robot please take out the junk
    # Meet Alex at the sink, follow him, and lead him back
    # Find Skyler at the sink and ask him to leave
    # Greet Robin at the bookcase and ask her to leave
    # Could you face Alex at the main door and introduce him to everyone in the dining room.
    # Please give me the right most object from the end table
    # Bring me the object above the knife from the dining table
    # Could you greet Skyler at the sink and ask her to leave
    # Go to the bookcase, meet Mary, and take her to the exit
    # Get the tray and place it on the desk
    # Tell me how many people in the living room are boys
    # Say your team's country to the person raising their right arm in the living room
    # Could you please give me the paprika from the end table
    # Tell me the gender of the person in the kitchen
    # Robot please tell me which are the three biggest snacks on the counter
    # Meet John at the couch, follow him, and navigate to the bedroom
    # Go to the bookcase, meet Patricia, and follow her
    # Robot please go to the dining table, meet Michael, and take him to the entrance
    # """
    #     check = set()
    #     for i in longmf.split('\n'):
    #         check.add(post_request("navigate",i,force_green=True))
    #     print(check)

