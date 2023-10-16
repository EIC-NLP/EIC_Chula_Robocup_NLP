# #! stt-https 2.0 - Speech Recognition Server

# Custom Libraries
from ratfin import *
from config import *

# Standard Libraries
import io
from pydub import AudioSegment
import speech_recognition as sr
from flask import Flask, request, jsonify
import requests
import os
from datetime import datetime
from scipy.io import wavfile
import noisereduce as nr
import simpleaudio as sa
import time
from src_client_pkg.nlp_client.response import Response


class SttServer:
    def __init__(self,
                 deploymode=False,
                 model="small",
                 energy=1200,
                 dynamic_energy=False,
                 pause=1,
                 rasa_nlu: bool = True,
                 always_listen: bool = False):

        printclr(f"STT...init whisper {model}", "magenta")
        import whisper  # this takes the longest to import

        # Init Flask
        self.app = Flask("stt_stt_flask")
        self.app.add_url_rule("/", "listen", self.listen, methods=["POST"])
        self.app.add_url_rule("/live_listen", "live_listen", self.live_listen, methods=["POST"])


        # General variable
        self.body = None
        self.always_listen = always_listen

        self.model = model

        # stt Variables
        self.energy = energy
        self.pause = pause
        self.sample_rate = 48000

        self.rasa_nlu = rasa_nlu
        self.deploymode = deploymode
        self.voice_path = os.path.join("stt/temp", "temp.wav")

        # check from config.py
        if default_mic:
            self.mic_index = None
            self.mic_name = "Default Microphone"
        else:
            self.mic_index = stt_mic_index
            self.mic_name = stt_mic_name


        # if always_listen:
        #     stt_url = always_stt_url

        # Load stt model
        self.audio_model = whisper.load_model(
            name = self.model,
            download_root="stt/model"
            )

        # Load stt Microphone
        self.r = sr.Recognizer()
        self.r.pause_threshold = pause
        self.r.dynamic_energy_threshold = dynamic_energy

        # Calibration for the mic if needed
        if calibration:
            with sr.Microphone(
                sample_rate=self.sample_rate,
                device_index=self.mic_index
                ) as source:

                printclr(f"Mic: {str(self.mic_index)} {self.mic_name}",
                         "magenta")
                printclr("Calibrating microphone...", "magenta")
                self.r.adjust_for_ambient_noise(source, duration=5)
                printclr(f"\t{calibration=}", "magenta")

                # Mulipler for the calibrated energy threshold
                self.r.energy_threshold *= calibration_coefficient

                printclr(f"\tAfterMulitplierOffset: {self.r.energy_threshold=}", "magenta")
        else:

            self.r.energy_threshold = energy_threshold
            printclr(f"\tUsing manual energy: {self.r.energy_threshold=}", "magenta")

        # Finished init
        printclr("stt...success", "magenta")


    # Run the stt server REST API
    def run(self):
        try:
            self.app.run(host="0.0.0.0",
                        port=int(stt_url.split(":")[-1][:-1:]),
                        threaded=True)
            print("\033[0;35m" + f"\nlisten(GET): {stt_url}" +
                "\n\033[0m")
        except OSError:
            printclr("Port already in use, please change port in .env", "red")
            exit()


    def trigger():
        wave_obj = sa.WaveObject.from_wave_file('trigger.wav')
        play_obj = wave_obj.play()
        play_obj.wait_done()  # Wait until sound has finished playing


    def get_intent(self, text) -> str(dict()):
        """ Return a json in type string
        >>> repr(get_intent())
        '{"intent": "restaurant_order", "confidence": 0.9987571239471436, "text": "Can I have a cook or cola please?", "object": "cola"}'
        """

        r = requests.post(url=rasa_url,
                          json={
                              "sender": "bot",
                              "message": text
                          })
        if r.json() == []:
            printclr("Rasa return []","red")
            return '{"intent": "", "confidence": 0.0, "text": "Rasa Error"}'
        else:
            # print("getintent: ", r.json()[0]['text'])
            return r.json()[0]['text']

    # Predicting the input from Audio
    def stt_result(self, trigger=False) -> Response:
        printclr("listening...", "blue")
        with sr.Microphone(sample_rate=self.sample_rate, device_index=self.mic_index) as source:
            if trigger:
                trigger()
            audio = self.r.listen(source)
            data = io.BytesIO(audio.get_wav_data())
            audio_clip = AudioSegment.from_file(data)
            audio_clip.export(self.voice_path, format="wav")
            printclr("computing...", "yellow")

        if noise_reduction:
            # load data
            rate, data = wavfile.read(self.voice_path)

            # perform noise reduction
            reduced_noise = nr.reduce_noise(y=data, sr=rate)
            wavfile.write(f"{self.voice_path}_processed.wav", rate, reduced_noise)
            # run stt on processed audio
            result = self.audio_model.transcribe(f"{self.voice_path}_processed.wav", language='english')
        else:
            # run stt on processed audio
            result = self.audio_model.transcribe(self.voice_path, language='english')

        response = Response(text=result["text"])

        path_friendly = response.text.replace(" ", "_")

        if noise_reduction:
            logging_path = "stt/logs/" + datetime.now().strftime("%Y-%m-%d--%H:%M:%S_") + path_friendly + "_processed.wav"
            wavfile.write(logging_path, rate, reduced_noise)
        else:
            logging_path = "stt/logs/" + datetime.now().strftime("%Y-%m-%d--%H:%M:%S_") + path_friendly + ".wav"
            audio_clip.export(logging_path, format="wav")

        printclr("You said: " + response.text, "blue")
        return response.text


    def listen(self, trigger=False) -> dict[str, str]:
        try:
            stt_text = self.stt_result(trigger=trigger)

            # Parse the JSON from the request
            data = request.get_json()

            # Get the 'intent' value from the JSON
            intent = data.get('intent', None)

            # Check if intent is provided and is a boolean
            if intent is not None and isinstance(intent, bool):
                # Execute your code here
                # For instance, let's return a different message based on the intent value
                if intent:
                    # obj  = {"body": self.get_intent(stt_text)}
                    import ast
                    obj  = jsonify(ast.literal_eval(self.get_intent(stt_text)))

                else: # for no intent
                    # Create a JSON payload as string
                    payload = {"text": stt_text}
                    # payload = str({"text": stt_text})
                    # Return the JSON payload for 2 unfolding
                    obj = jsonify({"body": payload})

                print(f"stt Returning: {obj}")
                return obj

            else:
                return jsonify({"error": "Invalid or missing 'intent' in JSON payload"}), 400
        except Exception as e:
            return jsonify({"body": ""})

    def live_listen(self, trigger=None, noise_reduction=False, timeout=2):
        printclr("listening...", "blue")

        full_audio_data = io.BytesIO()
        last_audio_time = time.time()

        def callback(recognizer, audio):
            nonlocal last_audio_time
            # get the current time
            current_time = time.time()

            # if the audio energy is above the threshold, update the last_audio_time
            if audio.rms > self.r.energy_threshold:
                last_audio_time = current_time

            # real-time transcription
            data = io.BytesIO(audio.get_wav_data())
            audio_clip = AudioSegment.from_file(data)
            slice_result = self.audio_model.transcribe(data, language='english')
            printclr(f"Real-time transcription: {slice_result['text']}", "blue")

            # append audio data to the buffer
            full_audio_data.write(audio.get_wav_data())

        with sr.Microphone(sample_rate=self.sample_rate, device_index=self.mic_index) as source:
            if trigger:
                trigger()

            # start listening in the background
            stop_listening = self.r.listen_in_background(source, callback)

            # loop until a period of silence longer than timeout
            while time.time() - last_audio_time < timeout:
                time.sleep(0.1)

            # stop listening
            stop_listening(wait_for_stop=False)

        # save the full audio data
        full_audio_clip = AudioSegment.from_file(full_audio_data)
        full_audio_clip.export(self.voice_path, format="wav")

        printclr("computing...", "yellow")

        if noise_reduction:
            # load data
            rate, data = wavfile.read(self.voice_path)

            # perform noise reduction
            reduced_noise = nr.reduce_noise(y=data, sr=rate)
            wavfile.write(f"{self.voice_path}_processed.wav", rate, reduced_noise)

            # run stt on processed audio
            result = self.audio_model.transcribe(f"{self.voice_path}_processed.wav", language='english')
        else:
            # run stt on processed audio
            result = self.audio_model.transcribe(self.voice_path, language='english')

        response = Response(text=result["text"])
        printclr(f"Final transcription: {response.text}", "blue")

        return response.text



if __name__ == "__main__":
    stt = SttServer(
        rasa_nlu=True,
        model= stt_model
    )
    stt.run()

    # stt.listen()


_MODELS = {
        "tiny.en":
        "https://openaipublic.azureedge.net/main/whisper/models/d3dd57d32accea0b295c96e26691aa14d8822fac7d9d27d5dc00b4ca2826dd03/tiny.en.pt",
        "tiny":
        "https://openaipublic.azureedge.net/main/whisper/models/65147644a518d12f04e32d6f3b26facc3f8dd46e5390956a9424a650c0ce22b9/tiny.pt",
        "base.en":
        "https://openaipublic.azureedge.net/main/whisper/models/25a8566e1d0c1e2231d1c762132cd20e0f96a85d16145c3a00adf5d1ac670ead/base.en.pt",
        "base":
        "https://openaipublic.azureedge.net/main/whisper/models/ed3a0b6b1c0edf879ad9b11b1af5a0e6ab5db9205f891f668f8b0e6c6326e34e/base.pt",
        "small.en":
        "https://openaipublic.azureedge.net/main/whisper/models/f953ad0fd29cacd07d5a9eda5624af0f6bcf2258be67c92b79389873d91e0872/small.en.pt",
        "small":
        "https://openaipublic.azureedge.net/main/whisper/models/9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794/small.pt",
        "medium.en":
        "https://openaipublic.azureedge.net/main/whisper/models/d7440d1dc186f76616474e0ff0b3b6b879abc9d1a4926b7adfa41db2d497ab4f/medium.en.pt",
        "medium":
        "https://openaipublic.azureedge.net/main/whisper/models/345ae4da62f9b3d59415adc60127b97c714f32e89e936602e85993674d08dcb1/medium.pt",
        "large-v1":
        "https://openaipublic.azureedge.net/main/whisper/models/e4b87e7e0bf463eb8e6956e646f1e277e901512310def2c24bf0e11bd3c28e9a/large-v1.pt",
        "large-v2":
        "https://openaipublic.azureedge.net/main/whisper/models/81f7c96c852ee8fc832187b0132e569d6c3065a3252ed18e56effd0b6a73e524/large-v2.pt",
    }