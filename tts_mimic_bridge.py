# This program is just a bridge
# To run the server
# mimic3-server --preload-voice en_US/vctk_low

import simpleaudio as sa
import requests
from ratfin import printclr
from flask import Flask, request


# crate server
app = Flask("tts_mimic_bidge_flask")

@app.route("/")
def home():
    return "home"

@app.route("/tts", methods=["POST","GET"])
def speaklocalapi(text):
    try:
      ssml = '''<speak>
      <voice name="en_US/vctk_low#p260">
      <prosody rate='0.8'>
        <s>
          TEXTTOBEREPLACED
        </s>
      </prosody>
      </voice>
    </speak>'''
      headers = {'Content-Type': 'application/ssml+xml'}
      ssml = ssml.replace("TEXTTOBEREPLACED", text)
      # print(ssml)
      response = requests.post(url='http://localhost:59125/api/tts',
                              headers=headers,
                              data=ssml)
      with open('output.wav', 'wb') as f:
          f.write(response.content)

      #play the audio file
      wave_obj = sa.WaveObject.from_wave_file('output.wav')
      play_obj = wave_obj.play()
      play_obj.wait_done()

      printclr('synthesized": f"{text}', "blue")

      return {"synthesized": f"{text}"}
    
    except Exception as e:
        printclr(e, "red")

#* running server tts
app.run(host="localhost", port=5003)


#main

# speak("Hello! My name is Walkie. I am ready to help you")
# speak('I could not understand what you said. Please say that again')