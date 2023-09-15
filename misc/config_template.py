#
#* General
verbose = True
default_mic = True # on or off


#* stt 
always_stt_url =  "http://localhost:5102/"
stt_url =  "http://localhost:5101/"
stt_model = "medium.en"

stt_mic_index= 4
stt_mic_name='USB Audio Device'

stt_energy_calibration =  True #
calibration = True
noise_reduction = True
energy_threshold =  1950
calibration_coefficient =  1.5


#* Wakeword
wakeword_url =  "http://localhost:5100/"

wakeword_key = "A60C/RHA8NoFcfih1VnnW+MR736qtLekdo/2DrKSTx3dQnd4kmZBlw=="

mic_index_ww =  0
mic_name_ww = "Built-in Microphone"


#* TTS
tts_url =  "http://localhost:5003/tts"

azure_key = "fd196d9bb49147e2bde3874c1447512e"


#* RASA NLU 
rasa_url =  "http://localhost:5005/webhooks/rest/webhook"
rasa_parse_url =  "http://localhost:5005/model/parse"
rasa_actions_url =  "http://localhost:5055"

gpt_key = "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"

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



