import azure.cognitiveservices.speech as speechsdk
from ratfin import printclr
from flask import Flask, request
from config import azure_key

#https://docs.microsoft.com/en-us/azure/cognitive-services/speech-service/language-support?tabs=speechtotext#prebuilt-neural-voices

region = "southeastasia"
printclr(f"\t{azure_key=}", "green")

list_voices = [
    "en-US-AmberNeural", #4
    "en-US-AriaNeural",#5
    "en-US-DavisNeural",
    "en-US-GuyNeural",
    "en-US-JennyNeural",
    "en-US-RogerNeural",
    "en-US-SaraNeural",
    "en-US-SteffanNeural",
    "th-TH-AcharaNeural",
    "th-TH-NiwatNeural",
    "th-TH-PremwadeeNeural"
]

try:
    speech_config = speechsdk.SpeechConfig(subscription=azure_key, region=region)
    audio_config = speechsdk.audio.AudioOutputConfig(use_default_speaker=True)
    # speech_config.speech_synthesis_voice_name ="en-US-JaneNeural"
    speech_config.speech_synthesis_voice_name ='en-US-JennyNeural'
    speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=audio_config)

except Exception as e:
    printclr(e,"red")
    

app = Flask("tts_azure")

@app.route("/")
def home():
    return "home"

@app.route("/tts", methods=["POST","GET"])
def speak(
    text: str="Deafault Response" , 
    voice : str = "en-US-JaneNeural" , 
    style : str = "normal" ,
    log : bool = False , 
    # profanity : str ="2" 
    ):
    try:
        x = request.get_json()
        # printclr("synthesizing...","blue")
        text = x['text']
        voice = x['voice']
        style = x['style']
        profanity = x['profanity']
    except:
        printclr("going with default values", "yellow")
        

    try:
        # Read XML
        ssml_string = open("tts/tts_voice_config.xml", "r").read()
        ssml_string = ssml_string.replace('TEXT', text)
        ssml_string = ssml_string.replace('STYLE', repr(style))
        ssml_string = ssml_string.replace('VOICE', repr(voice))

        #* Speech Config
        # speech_config.set_profanity = profanity
        # speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=audio_config)
        
        # sending & receiver from Azure
        result = speech_synthesizer.speak_ssml_async(ssml_string).get()
        printclr(f"\tsynthesized: {text=}", "blue")

        #* Convert into audio stream
        # stream = speechsdk.AudioDataStream(result)
        # stream.save_to_wav_file("tts_temp.wav")


        return {"synthesized": f"{text}"}
    
    except Exception as e:
        printclr(e, "red")


def test():
    for i in list_voices:
        speech_config = speechsdk.SpeechConfig(subscription=azure_key, region=region)
        audio_config = speechsdk.audio.AudioOutputConfig(suse_default_speaker=True)

        # The language of the voice that speaks.
        #speech_config.speech_synthesis_voice_name='en-US-JennyNeural'
        # speech_config.speech_synthesis_voice_name= list_voices[0]
        speech_config.speech_synthesis_voice_name = i
        speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=audio_config)
        print(i)

        text = "Hi there my name is Walkie, I will be your personal assistant"

        speech_synthesis_result = speech_synthesizer.speak_text_async(text).get()

        if speech_synthesis_result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            printclr(f"\tsynthesized.", "blue")

        elif speech_synthesis_result.reason == speechsdk.ResultReason.Canceled:
            cancellation_details = speech_synthesis_result.cancellation_details
            print("Speech synthesis canceled: {}".format(cancellation_details.reason))
            if cancellation_details.reason == speechsdk.CancellationReason.Error:
                if cancellation_details.error_details:
                    print("Error details: {}".format(cancellation_details.error_details))
                    print("Did you set the speech resource key and region values?")

    
#* testing 
# speak(text="Thank you, now get your arse in the shower")

#* running server tts
app.run(host="localhost", port=5003)