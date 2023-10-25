import pvporcupine
from pvrecorder import PvRecorder
from dotenv import load_dotenv
import os

load_dotenv()


from config import mic_index_ww, stt_url, wakeword_url, default_mic
from flask import Flask,request
from ratfin import *
import platform

porcupine_key = os.getenv('PORCUPINE_KEY')

print(f'Your API key is: {porcupine_key}')

if platform.system() == "Darwin":
    platform_ = "MacOS"
    hey_walkie_path = ["wakeword/resources/keyword_files/mac/Hey-Walkie_en_mac_v2_2_0.ppn"]
    walkie_freeze_path = ['wakeword/resources/keyword_files/linux/Walkie-Freeze_en_linux_v2_2_0.ppn']
else:
    platform_ = platform.system()
    hey_walkie_path = ['wakeword/resources/keyword_files/linux/Hey-Walkie_en_linux_v2_2_0.ppn']
    walkie_freeze_path = ['wakeword/resources/keyword_files/linux/Walkie-Freeze_en_linux_v2_2_0.ppn']


def listen_for_hey_walkie():
    try:
        porcupine = pvporcupine.create(access_key=porcupine_key, keyword_paths=hey_walkie_path)
        recorder = PvRecorder(device_index=-1, frame_length=porcupine.frame_length)
        printclr("Listening...", "blue")
        recorder.start()
        while True:
            try:
                keyword_index = porcupine.process(recorder.read())
                if keyword_index >= 0:
                    # print("detected keyword")
                    recorder.stop()
                    return {'status': 'success', 'message': 'wakeword detected'}
            except e:
                listen_for_hey_walkie()
    except pvporcupine.PorcupineInvalidArgumentError as e:
        print("One or more arguments provided to Porcupine is invalid")
        raise e
    except pvporcupine.PorcupineActivationError as e:
        print("AccessKey activation error")
        raise e
    except pvporcupine.PorcupineActivationLimitError as e:
        print("AccessKey has reached it's temporary device limit")
        raise e
    except pvporcupine.PorcupineActivationRefusedError as e:
        print("AccessKey refused")
        raise e
    except pvporcupine.PorcupineActivationThrottledError as e:
        print("AccessKey has been throttled")
        raise e
    except pvporcupine.PorcupineError as e:
        print("Failed to initialize Porcupine")
        raise e
    except KeyboardInterrupt:
        recorder.stop()

def listen_for_walkie_freeze():
    try:
        porcupine = pvporcupine.create(access_key=porcupine_key, keyword_paths=walkie_freeze_path)
        recorder = PvRecorder(device_index=-1, frame_length=porcupine.frame_length)
        printclr("Listening...", "blue")
        recorder.start()
        while True:
            try:
                keyword_index = porcupine.process(recorder.read())
                if keyword_index >= 0:
                    # print("detected keyword")
                    recorder.stop()
                    return {'status': 'success', 'message': 'wakeword detected'}
            except e:
                listen_for_hey_walkie()
    except pvporcupine.PorcupineInvalidArgumentError as e:
        print("One or more arguments provided to Porcupine is invalid")
        raise e
    except pvporcupine.PorcupineActivationError as e:
        print("AccessKey activation error")
        raise e
    except pvporcupine.PorcupineActivationLimitError as e:
        print("AccessKey has reached it's temporary device limit")
        raise e
    except pvporcupine.PorcupineActivationRefusedError as e:
        print("AccessKey refused")
        raise e
    except pvporcupine.PorcupineActivationThrottledError as e:
        print("AccessKey has been throttled")
        raise e
    except pvporcupine.PorcupineError as e:
        print("Failed to initialize Porcupine")
        raise e
    except KeyboardInterrupt:
        recorder.stop()




app = Flask("wakeword_porcupine_flask")
# app.add_url_rule("/", "index", check_status)
app.add_url_rule("/hey_walkie", "listen_for_hey_walkie", listen_for_hey_walkie)
app.add_url_rule("/walkie_freeze", "listen_for_walkie_freeze", listen_for_walkie_freeze)

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0",
                port=int(wakeword_url.split(":")[-1][:-1:]),
                threaded=True)
        print("\033[0;35m" + f"\nlisten(GET): {wakeword_url}" + "\n\033[0m")
    except OSError:
        printclr("Port already in use, please change port in nlp_config.json",
                    "red")
        exit()


    # listen()
    # listen_for_hey_walkie()
    # listen_for_walkie_freeze()


