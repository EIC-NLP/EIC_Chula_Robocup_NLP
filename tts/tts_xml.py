import azure.cognitiveservices.speech as speechsdk

speech_config = speechsdk.SpeechConfig(
    subscription='44d7d58744334bea8b7c72de640ed3e3', region='southeastasia')
audio_config = speechsdk.audio.AudioOutputConfig(use_default_speaker=True)


def speakText(text):

    # speech_config.speech_synthesis_voice_name = "fr-FR-JeromeNeura"
    speech_config.speech_synthesis_voice_name = "en-US-JaneNeural"
    speech_config.set_profanity = 2

    speech_synthesizer = speechsdk.SpeechSynthesizer(
        speech_config=speech_config, audio_config=audio_config)
    speech_synthesis_result = speech_synthesizer.speak_text_async(text).get()


def speakTextSSML(
    text: str="Deafault Response", 
    voice : str ='"en-US-JaneNeural"', 
    style : str ='"normal"', 
    profanity : str ="2" 
    ):
    # Private variables
    speech_config.set_profanity = profanity
    
    # Read XML
    ssml_string = open("tts/tts_voice_config.xml", "r").read()
    ssml_string = ssml_string.replace('TEXT', text)
    ssml_string = ssml_string.replace('STYLE', repr(style))
    ssml_string = ssml_string.replace('VOICE', repr(voice))
    
    print(ssml_string)

    synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config,
                                              audio_config=audio_config)
    result = synthesizer.speak_ssml_async(ssml_string).get()

    stream = speechsdk.AudioDataStream(result)
    
    stream.save_to_wav_file("tts_temp.wav")


if __name__ == "__main__":
    speakTextSSML(
        text = "YO MOTHERFUCKER, WAKE THE FUCK UP,WAKE THE FUCK UP,WAKE THE FUCK UP,WAKE THE FUCK UP,WAKE THE FUCK UP",
        voice = "en-US-JaneNeural" ,
        style = "shouting" # can be shouting, normal

    )
