# Custom Libraries
from termcolor import colored
from config import (
    calibration,
    calibration_coefficient,
    energy_threshold,
    noise_reduction,
    stt_url,
)

# Standard Libraries
import io
from pydub import AudioSegment
import speech_recognition as sr
from flask import Flask, jsonify
import os
from datetime import datetime
from scipy.io import wavfile
import simpleaudio as sa
import noisereduce as nr
import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline


class SttServer:
    def __init__(
        self,
        deploymode=False,
        energy=1200,
        dynamic_energy=False,
        rasa_nlu: bool = True,
        always_listen: bool = False,
    ):
        # Init Flask
        self.app = Flask("distil_whisper")
        self.app.add_url_rule("/", "listen", self.listen, methods=["POST"])

        # General variable
        self.body = None
        self.rasa_nlu = rasa_nlu

        # Configurations
        self.model = "distil-whisper/distil-medium.en"
        # self.model = "distil-whisper/distil-large-v2"
        self.energy = energy
        self.pause = 1  # second
        self.sample_rate = 48000
        self.voice_path = os.path.join("stt/temp", "temp.wav")
        self.mic_index = None
        self.mic_name = "Default Microphone"

        # Load stt Microphone
        self.r = sr.Recognizer()
        self.r.pause_threshold = self.pause
        self.r.dynamic_energy_threshold = dynamic_energy

        # Calibration for the mic if needed
        if calibration:
            with sr.Microphone(
                sample_rate=self.sample_rate, device_index=self.mic_index
            ) as source:
                print(colored(f"Mic: {str(self.mic_index)} {self.mic_name}", "magenta"))
                print(colored("Calibrating microphone...", "magenta"))
                self.r.adjust_for_ambient_noise(source, duration=5)
                print(colored(f"\t{calibration=}", "magenta"))

                # Mulipler for the calibrated energy threshold
                self.r.energy_threshold *= calibration_coefficient

                print(colored(
                    f"\tAfterMulitplierOffset: {self.r.energy_threshold=}", "magenta"
                ))
        else:
            self.r.energy_threshold = energy_threshold
            print(colored(f"\tUsing manual energy: {self.r.energy_threshold=}", "magenta"))

        print(colored(f"STT...init whisper {self.model}", "magenta"))
        if torch.cuda.is_available():
            self.torch_dtype = torch.float16
            self.device = "cuda:0"
            print("Using CUDA acceleration")
        elif torch.backends.mps.is_available():
            self.torch_dtype = torch.float16
            self.device = "mps"
            print("Using MPS acceleration")
        else:
            self.torch_dtype = torch.float32
            self.device = "cpu"
            print("Using CPU Float32")

        ## init the model
        self.distil_model = AutoModelForSpeechSeq2Seq.from_pretrained(
            self.model,
            torch_dtype=self.torch_dtype,
            low_cpu_mem_usage=True,
            use_safetensors=True,
        ).to(self.device)
        # self.distil_model.to(self.device)

        self.processor = AutoProcessor.from_pretrained(self.model)

        self.pipeline = pipeline(
            "automatic-speech-recognition",
            model=self.model,
            tokenizer=self.processor.tokenizer,
            feature_extractor=self.processor.feature_extractor,
            max_new_tokens=128,
            chunk_length_s=15,
            batch_size=16,
            torch_dtype=self.torch_dtype,
            device=self.device,
        )
        # Finished init
        print(colored("stt...success", "magenta"))

    # Run the stt server REST API
    def run(self):
        try:
            self.app.run(
                host="0.0.0.0", port=int(stt_url.split(":")[-1][:-1:]), threaded=True
            )
            print("\033[0;35m" + f"\nlisten(GET): {stt_url}" + "\n\033[0m")
        except OSError:
            print(colored("Port already in use, please change port in .env", "red"))
            exit()

    def listen(self, trigger=False) -> dict[str, str]:
        try:
            # Record
            with sr.Microphone(
                sample_rate=self.sample_rate, device_index=self.mic_index
            ) as source:
                if trigger:
                    trigger()
                print(colored("listening...", "blue"))
                audio = self.r.listen(source)
                data = io.BytesIO(audio.get_wav_data())
                audio_clip = AudioSegment.from_file(data)
                audio_clip.export(self.voice_path, format="wav")
                print(colored("computing...", "yellow"))
            # Processing
            if noise_reduction:
                # load data
                rate, data = wavfile.read(self.voice_path)

                # perform noise reduction
                reduced_noise = nr.reduce_noise(y=data, sr=rate)
                wavfile.write(f"{self.voice_path}_processed.wav", rate, reduced_noise)
                # run stt on processed audio

                transcribed_text_dict = self.pipeline(
                    f"{self.voice_path}_processed.wav"
                )
            else:
                # run stt on processed audio
                transcribed_text_dict = self.pipeline(f"{self.voice_path}.wav")

            transcribed_text = transcribed_text_dict["text"]

            print(colored("You said: " + transcribed_text, "blue"))

            # Logging
            path_friendly = transcribed_text.replace(" ", "_")

            if noise_reduction:
                logging_path = (
                    "stt/logs/"
                    + datetime.now().strftime("%Y-%m-%d--%H:%M:%S_")
                    + path_friendly
                    + "_processed.wav"
                )
                wavfile.write(logging_path, rate, reduced_noise)
            else:
                logging_path = (
                    "stt/logs/"
                    + datetime.now().strftime("%Y-%m-%d--%H:%M:%S_")
                    + path_friendly
                    + ".wav"
                )
                audio_clip.export(logging_path, format="wav")

            # contain the text in a response object
            return transcribed_text

        except Exception as e:
            log_exception_with_traceback(e)
            return jsonify({"error": "An exception occurred: " + str(e)}), 500

    def _trigger():
        wave_obj = sa.WaveObject.from_wave_file("trigger.wav")
        play_obj = wave_obj.play()
        play_obj.wait_done()  # Wait until sound has finished playing


if __name__ == "__main__":
    stt = SttServer(
        rasa_nlu=True,
    )
    stt.run()

    # stt.listen()
