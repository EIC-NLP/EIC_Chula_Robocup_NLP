from nlp_client import *
from datetime import datetime

speak(
        text=
        "Good Morning, it's %s, You have an appointment in 1 hour, you need to get up"
        % datetime.now().strftime("%H:%M"),
        style="shouting"
        )
print("getfucked")

speak(text="Thank you, now get your arse in the shower", style="shouting")
