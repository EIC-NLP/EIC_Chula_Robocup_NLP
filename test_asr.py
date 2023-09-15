from asr_server import AsrServer

model = "tiny.en"
stt = AsrServer( rasa_nlu=True, model= model)

while True:
    stt.live_listen()
