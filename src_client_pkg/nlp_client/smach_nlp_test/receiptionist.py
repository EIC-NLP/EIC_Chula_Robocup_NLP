from nlp_client import *
import time
from util.guest_name_manager import GuestNameManager

person_count = 0 
gm = GuestNameManager("smach_nlp_test/receptionist_database.yaml")
gm.reset()

class Ask():
    def __init__(self):
        speak("What is your name?")
        person_name = ""

        while True:
            res_listen = listen()
            if res_listen["intent"] == "my_name" and 'people' in res_listen:
                break
            else:
                speak("Sorry I don't understand, Could you rephrase that?")
        
        person_name = res_listen["people"]
        print(person_name)
        gm.add_guest_name("guest_{}".format(person_count), person_name)


        # register face
        speak("Please show your face to the robot's camera")

            # count down
        for number in ["three", "two", "one"]:
            speak(number)
            time.sleep(1.0)
        speak("capture!")

            # scale image incase image size donot match cv server
        
        # listening to the person and save his his/her fav_drink to the file
        speak("What is your favorite drink?")
        object_name = ""

        while True:
            res_listen = listen()
            if res_listen["intent"] == "favorite_object" and 'object' in res_listen:
                break
            else:
                speak("Sorry I don't understand, Could you rephrase that?")
        object_name = res_listen["people"]
        print(object_name)
        gm.add_guest_name("guest_{}".format(person_count), object_name)

        if person_count == 1:
            speak("Hello {host_name}, the guest who is on the {furniture} is {guest_1}".format(host_name = gm.get_guest_name("host"), furniture = "Couch", guest_1 = gm.get_guest_name("guest_1")))
            speak("His favorite drink is {fav_drink1}".format(fav_drink1 = gm.get_guest_fav_drink("guest_1")))
        if person_count == 2:
            speak("Hello {host_name}, the new guest is {guest_2}".format(host_name = gm.get_guest_name("host"), guest_2 = gm.get_guest_name("guest_2")))
            speak("His favorite drink is {fav_drink2}".format(fav_drink2 = gm.get_guest_fav_drink("guest_2")))
        return 'continue_Introduce_host'

instance = Ask()