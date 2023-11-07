#
#* General
verbose = True
default_mic = True # on or off

#! CHAGE HERE FOR DISABLING THE CALIBRATION
calibration = True
# calibration = Falsew
energy_threshold =  746 # for quiet room for Game's Mac (Mic at 90%)
calibration_coefficient =  1.5

# LocalHost Servers
wakeword_url =  "http://localhost:5100/"
stt_url =  "http://localhost:5101/"
tts_url =  "http://localhost:5003/tts"
rasa_url =  "http://localhost:5005/webhooks/rest/webhook"
rasa_parse_url =  "http://localhost:5005/model/parse"
rasa_actions_url =  "http://localhost:5055"

#* stt
stt_model = "base.en"
stt_mic_index= 4
stt_mic_name='USB Audio Device'
stt_energy_calibration =  True #
noise_reduction = True

#* Wakeword
mic_index_ww =  0
mic_name_ww = "Built-in Microphone"

#* TTS


#* RASA NLU


"""
stt Notes
- Calibration: Auto calibrate first then grab the number
- list of Whisper Model
    "tiny.en"
    "tiny"
    "base.en"
    "base"
    "small.en"
    "small"
    "medium.en"
    "medium"
    "large-v1"
    "large-v2"

"""



