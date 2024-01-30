import pvporcupine
from pvrecorder import PvRecorder
from dotenv import load_dotenv
import jsonify
import traceback
import os


class bcolors:
    RED_FAIL = "\033[91m"
    GRAY_OK = "\033[90m"
    GREEN_OK = "\033[92m"
    YELLOW_WARNING = "\033[93m"
    BLUE_OK = "\033[94m"
    MAGENTA_OK = "\033[95m"
    CYAN_OK = "\033[96m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"


load_dotenv()

from config import mic_index_ww, stt_url, wakeword_url, default_mic
from flask import Flask, request
import platform

# * You can get the API key from https://console.picovoice.ai/
porcupine_key = os.getenv("PORCUPINE_KEY")

# print(f'Your API key is: {porcupine_key}')


if platform.system() == "Darwin":
    platform_ = "MacOS"
    wakewords = {
        "hey_walkie": "wakeword/resources/keyword_files/mac/Hey-Walkie_en_mac_v2_2_0.ppn",
        # There's no EmergencyStop model for mac
    }
else:
    platform_ = platform.system()

    wakewords = {
        "hey_walkie": "wakeword/resources/keyword_files/linux/Hey-Walkie_en_linux_v2_2_0.ppn",
        "walkie_freeze": "wakeword/resources/keyword_files/linux/Walkie-Freeze_en_linux_v2_2_0.ppn",
    }


class WakeWordServer:
    def __init__(
        self,
    ):
        def _crate_wakeword_instances():
            wakeword_instances = {}
            for wakeword, model_path in wakewords.items():
                try:
                    porcupine = pvporcupine.create(
                        access_key=porcupine_key, keyword_paths=[model_path]
                    )
                    recorder = PvRecorder(
                        device_index=-1, frame_length=porcupine.frame_length
                    )
                    wakeword_instances[wakeword] = {
                        "porcupine": porcupine,
                        "recorder": recorder,
                    }
                    print(bcolors.GRAY_OK + f"created instances of {wakeword_instances.keys()}" + bcolors.ENDC)
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
            return wakeword_instances

        # Init Flask
        self.app = Flask("wakeword_porcupine_flask")
        self.app.add_url_rule(
            "/<wakeword>",
            "listen_for_wakeword",
            self.listen_for_wakeword_handler,
            methods=["GET", "POST"],
        )

        # Init Model
        self.wakeword_instances = _crate_wakeword_instances()

    def listen_for_wakeword_handler(self, wakeword):
        if wakeword not in self.wakeword_instances:
            # Handle the case where the wakeword is not recognized
            return {"error": f"Wakeword '{wakeword}' not recognized"}, 400

        try:
            print(f"Listening for {wakeword}")
            porcupine = self.wakeword_instances[wakeword]["porcupine"]
            recorder = self.wakeword_instances[wakeword]["recorder"]
            recorder.start()
            while True:
                try:
                    keyword_index = porcupine.process(recorder.read())
                    if keyword_index >= 0:
                        recorder.stop()
                        print(bcolors.BLUE_OK + f"Wakeword Detected" + bcolors.ENDC)
                        return {
                            "status": "success",
                            "message": "wakeword detected",
                        }  # Triggering the function associated with the wakeword
                except Exception as e:
                    tb_str = traceback.format_exc()
                    print(bcolors.RED_FAIL + tb_str + bcolors.ENDC)
                    return {"error": "An exception occurred: " + str(e)}
        except Exception as e:
            print(bcolors.RED_FAIL + "Error: " + str(e) + bcolors.ENDC)
            return {"error": "An initialization error occurred: " + str(e)}

    # Run the stt server REST API
    def run(self):
        try:
            self.app.run(
                host="0.0.0.0",
                port=int(wakeword_url.split(":")[-1][:-1:]),
                threaded=True,
            )
            print("\033[0;35m" + f"\nlisten(GET): {stt_url}" + "\n\033[0m")
        except OSError:
            print(
                bcolors.RED_FAIL
                + f"Port already in use, please change port in .env"
                + bcolors.ENDC
            )
            exit()


if __name__ == "__main__":
    server = WakeWordServer()
    server.run()