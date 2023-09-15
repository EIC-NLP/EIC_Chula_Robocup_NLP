# init import
import os
os.system("clear")
import json
from ratfin import *
from prettytable import PrettyTable
from nlp_client import *
# constants

import time

def interface(text):
    basicUI = PrettyTable()
    basicUI.field_names = [text] 
    print(basicUI)



""" config mic """
def config_mic():
    interface("Config Whisper Microphone")
    import speech_recognition as sr
    r = sr.Recognizer()

    my_table = PrettyTable()

    my_table.field_names = ["device_index", "name"]
    io_list = sr.Microphone.list_microphone_names()
    for i, name in enumerate(io_list):
        my_table.add_row([i, name])

    print(my_table)
    printclr("If there's a 'default option', use that. \n otherwise use the most default one'","yellow")
    print("which device_index do you want to use?")
    device_index = input("device_index: ")

    os.environ["mic_index"] = device_index
    os.environ["mic_name"] = io_list[int(device_index)]
    
    dotenv.set_key(dotenv_file, "mic_index", device_index)
    dotenv.set_key(dotenv_file, "mic_name", io_list[int(device_index)])
    clearterm()
    x1 = int(os.getenv("mic_index"))
    x2 = os.getenv("mic_name")
    printclr(f"\tmic_index={repr(x1)}","green")
    printclr(f"\tmic_name={repr(x2)}","green")
    printclr(f"\n\nnlp_config.json updated\n","green")
    
    # clearterm()
    # from wakeword.wakeword_porcupine import PorcupineDemo
    # interface("Config WakeWord Mic")
    # devices = PorcupineDemo.show_audio_devices(verbose=False)
    # # print(f"{devices=}")

    # my_table = PrettyTable()

    # my_table.field_names = ["device_index", "name"]
    # for i, name in enumerate(devices):
    #     my_table.add_row([i, name])
    # print(my_table)
    # print("which device_index do you want to use?")
    # device_index = int(input("device_index: "))
    # try:

    #     clearterm()
    #     with open('nlp_config.json', 'r') as f:
    #         # Reading from json file
    #         json_object = json.load(f)
            
    #     with open('nlp_config.json', 'w') as f:
    #         # print({"mic_device": [device_index,io_list[device_index]]})
    #         json_object.update({
    #             "mic_wake_word_index": device_index,
    #             "mic_wake_word_name": devices[device_index]
    #             })
    #         json_object = json.dumps(json_object, indent = 4)
    #         print('\033[0;36m' + "\n\nnlp_config.json updated\n" + '\033[0m')
    #         print(json_object)
    #         f.write(json_object)
    # except:
    #     print("Error during reading nlp_config.json, using default mic")
    #     print("\n\n\ttry to create a new file nlp_config.json with the following content:\n\t{   }\n")
    

def get_range_energy():
    ambient_energy = int(0)
    speech_energy = int(0)

    mic_index = int(os.getenv("mic_index"))
    mic_name = os.getenv("mic_name")
    printclr(f"\tmic_index={repr(mic_index)}","green")
    printclr(f"\tmic_name={repr(mic_name)}","green")
    
    import speech_recognition as sr 
    r = sr.Recognizer() # load the app  
    speak("Please be quiet")
    time.sleep(1.5)
    with sr.Microphone(sample_rate=16000, device_index=mic_index) as source:
        printclr(f"Mic: {str(mic_index)} {mic_name}", "magenta")
        printclr("Please wait. Calibrating microphone...", "magenta")
        r.adjust_for_ambient_noise(source, duration=3)
        printclr(f"\tAmbient: {r.energy_threshold=}\n","magenta")
    r = sr.Recognizer() # load the app  
    speak("say something.")
    time.sleep(1.5)
    with sr.Microphone(sample_rate=16000, device_index=mic_index) as source:
        printclr(f"Mic: {str(mic_index)} {mic_name}", "magenta")
        printclr("Please wait. Calibrating microphone...", "magenta")
        r.adjust_for_ambient_noise(source, duration=3)
        printclr(f"\Speech: {r.energy_threshold=}\n","magenta")


""" config keyword """
def config_keyword():
    interface("Setuptool for EIC NLP")
    print("")

def get_socket_name():
    interface("Get name for socket config")
    import os
    # path = os.path.dirname(os.path.realpath(__file__))
    import platform
    if platform.system() == "Darwin":
        platform_ = "MacOS"
    else:
        platform_ = platform.system()
    path = f"socketconfig_{platform_}_{os.getlogin( )}.yaml"
    printclr(path, "green")
    print(f"\n Above is the name that your socketconfig file must use to ensure no error for path in different computer")

def main():
    clearterm() 
    functions = [config_mic, get_socket_name, get_range_energy]
    print("Setup tool for EIC NLP")
    print("1. config mic")
    print("2. get_socket_name")
    print("3. get_range_energy")
    print("4. exit")
    choice = int(input("choice: ")) -1
    clearterm()
    if choice == len(functions):
        exit()
    functions[choice]()

if __name__ == "__main__":
    main()
# %%
