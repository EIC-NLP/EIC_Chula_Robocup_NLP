from nlp_client import * 
from ratfin import printclr

# printclr("testing normal stt with intent","yellow")
# x = listen(intent=True, log=True)
# printclr(x.text,'blue')

# printclr("testing normal stt without intent","yellow")
x = listen(intent=False, log=True)
printclr(x.text,'blue')

# printclr("testing TTS Azure","yellow")
# speak("Hello there my name is Game",online=True)

# printclr("testing TTS Mimic","yellow")
# speak("Hello there. my name is Game",online=False)
