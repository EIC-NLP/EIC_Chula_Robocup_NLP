# run with conda env: nlp
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
import signal
import time
import os
from ratfin import *
from person import Person
from nlp_utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)


class EmergencyStop():
    """ How to use
    1. Create an instance of EmergencyStop
    2. Create a thread to run the execute function
    3. Call the emer_stop_handler function in the main thread
    
    >>> es = EmergencyStop()
    >>> es_thread = threading.Thread(target=es.execute)
    >>> es_thread.start()
    >>> es.emer_stop_handler() 
    """
    def __init__(self):
        self.stop_flag = False


    def execute(self):
        print('(EmergencyStop): Listening for Walkie Freeze')
        if nlp_client.ww_listen(text="walkie_freeze", log=True):
            self.stop_flag = True

        print('EmergencyStop detected')

        return "out1"
    def emer_stop_handler(self):
        while True:
            pid = os.getpid()
            if self.stop_flag:
                print("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
                pid = pid  # Replace with your process id
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    print(f"Process with id {pid} does not exist.")
                except PermissionError:
                    print(f"You don't have permission to kill process with id {pid}.")
                break
            time.sleep(0.1)


# function to run emergencystop
def listen_for_kill_command():
    """ 
    Listens for "Walkie Freeze" this will kill the program """
    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()
    return True

def main():

    x = EmergencyStop()

    x.execute()

    
if __name__ == '__main__':
    main()
