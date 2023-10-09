#
#* General
verbose = True
default_mic = True # on or off

#! CHAGE HERE FOR DISABLING THE CALIBRATION
# calibration = True
calibration = False
energy_threshold =  711
calibration_coefficient =  1.5

# LocalHost Servers
wakeword_url =  "http://localhost:5100/"
asr_url =  "http://localhost:5101/"
tts_url =  "http://localhost:5003/tts" # Azure
# tts_url =  'http://localhost:59125/api/tts' # mimic
rasa_url =  "http://localhost:5005/webhooks/rest/webhook"
rasa_parse_url =  "http://localhost:5005/model/parse"
rasa_actions_url =  "http://localhost:5055"

#* ASR
asr_model = "medium.en"
asr_mic_index= 4
asr_mic_name='USB Audio Device'
asr_energy_calibration =  True #
noise_reduction = True

#* Wakeword
wakeword_key = "A60C/RHA8NoFcfih1VnnW+MR736qtLekdo/2DrKSTx3dQnd4kmZBlw=="
mic_index_ww =  0
mic_name_ww = "Built-in Microphone"

#* TTS
azure_key = "fd196d9bb49147e2bde3874c1447512e"


#* RASA NLU

gpt_key = "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"

"""
ASR Notes
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



