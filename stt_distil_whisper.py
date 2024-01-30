# Custom Libraries
from config import (
    calibration,
    calibration_coefficient,
    energy_threshold,
    noise_reduction,
    stt_url,
)

# Standard Libraries
import io
import traceback
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
                print(
                    bcolors.GREEN_OK
                    + f"Mic: {str(self.mic_index)} {self.mic_name}"
                    + bcolors.ENDC
                )
                print(bcolors.GREEN_OK + bcolors.BOLD + f"Calibration On: " + bcolors.ENDC)
                print(bcolors.GREEN_OK + f"Calibrating microphone..." + bcolors.ENDC)
                self.r.adjust_for_ambient_noise(source, duration=5)
                print(bcolors.GREEN_OK + f"{calibration=}" + bcolors.ENDC)

                # Mulipler for the calibrated energy threshold
                self.r.energy_threshold *= calibration_coefficient

            print(
                bcolors.GREEN_OK + bcolors.BOLD + f"Calibration: " + bcolors.ENDC +"\n"
                + f"\tAfterMulitplierOffset: {self.r.energy_threshold=}"
                + bcolors.ENDC
            )
        else:
            self.r.energy_threshold = energy_threshold
            print(
                bcolors.MAGENTA_OK + bcolors.BOLD + f"Calibration Off: " + bcolors.ENDC
            )
            print(
                bcolors.YELLOW_WARNING + f"# For macbook: for 2500 level put mic at 90% input volume" + bcolors.ENDC
            )
            print(
                bcolors.MAGENTA_OK
                + f"\tUsing manual fixed energy: {self.r.energy_threshold=}"
                + bcolors.ENDC
            )

        print(bcolors.GRAY_OK+ f"STT...init   |   distil-whisper {self.model}" + bcolors.ENDC)
        if torch.cuda.is_available():
            self.torch_dtype = torch.float16
            self.device = "cuda:0"
            print(bcolors.GRAY_OK+ f"Using CUDA acceleration" + bcolors.ENDC)
        elif torch.backends.mps.is_available():
            self.torch_dtype = torch.float16
            self.device = "mps"
            print(bcolors.GRAY_OK+ f"Using MPS acceleration" + bcolors.ENDC)
        else:
            self.torch_dtype = torch.float32
            self.device = "cpu"
            print(bcolors.GRAY_OK+ f"Using CPU Float32" + bcolors.ENDC)

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
        print(bcolors.GREEN_OK + f"STT...Ready!" + bcolors.ENDC)

    # Run the stt server REST API
    def run(self):
        try:
            self.app.run(
                host="0.0.0.0", port=int(stt_url.split(":")[-1][:-1:]), threaded=True
            )
            print("\033[0;35m" + f"\nlisten(GET): {stt_url}" + "\n\033[0m")
        except OSError:
            print(bcolors.RED_FAIL + f"Port already in use, please change port in .env" + bcolors.ENDC)
            exit()

    def listen(self, trigger=False) -> dict[str, str]:
        try:
            # Record
            with sr.Microphone(
                sample_rate=self.sample_rate, device_index=self.mic_index
            ) as source:
                if trigger:
                    trigger()
                print(bcolors.BLUE_OK + f"listening..." + bcolors.ENDC)
                audio = self.r.listen(source)
                data = io.BytesIO(audio.get_wav_data())
                audio_clip = AudioSegment.from_file(data)
                audio_clip.export(self.voice_path, format="wav")
                print(bcolors.YELLOW_WARNING + f"computing..." + bcolors.ENDC)
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

            print(bcolors.BLUE_OK + f"You said: {transcribed_text}" + bcolors.ENDC)

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
            tb_str = traceback.format_exc()
            print(bcolors.RED_FAIL + tb_str + bcolors.ENDC)
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
