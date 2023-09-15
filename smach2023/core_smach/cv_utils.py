
""" 

Guidelines for writing a state:

Consult example_smach_state.py for an ideal state
--------------------------------------------
--MUST HAVE: 

1. OUTCOME INFORMATION: Have to declear all
    outcomes=['out1','out2','loop','undo','fail'] # out2 is optional, a good practice should have loop & undo

2. INPUT/OUTPUT DATA: Have to declare all
    input_keys=['data1', 'data2'],
    output_keys=['data1', 'data2']

2. LOGGING: EACH STEP MUST HAVE LOGGING TO ROSPY
    rospy.loginfo('Executing state State1')
    rospy.loginfo(f'({name of class state}}): Executing..')
    rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')

3. REMAPPING: ADD TO LIST FOR CONSTRUCTOR 
    # Will be added to the state machine by inputs and outputs
    remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

4. EXCEPTION HANDLING: return loop if exception or undo

5. FIX DATATYPE: NO DYNAMIC TYPE
    # Fix datatype to string, int, float, bool, list, dict
    log : bool = False # have default value
    # Raise exceptions if any entity parameter is not of type bool
    if not isinstance(intent, bool):
        raise ValueError("Argument 'intent' must be of type bool")
    if None then raise exception for variable CANNOT BE NONE

--------------------------------------------
--OPTIONAL:  

"""

import rospy
import smach
import nlp_client
import threading
from ratfin import *
import socket
import cv2
from cv_custom_socket import CustomSocket

from nlp_emerStop import listen_for_kill_command


# General CV Funcitons

def list_available_cam(max_n):
    list_cam = []
    for n in range(max_n):
        cap = cv2.VideoCapture(n)
        ret, _ = cap.read()

        if ret:
            list_cam.append(n)
        cap.release()
    
    if len(list_cam) == 1:
        return list_cam[0]
    else:
        print(list_cam)
        return int(input("Cam index: "))






# Model Smach States
class DetectFace(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """

    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        self.log = log
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Setup CV Capture

        self.cap = cv2.VideoCapture(list_available_cam(10))
        self.cap.set(4, 480)
        self.cap.set(3, 640)

        # Setup socket
        host = socket.gethostname()
        port = 12304
        self.c = CustomSocket(host, port, log=True)
        self.c.clientConnect()




    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(DetectFace): Executing..')

            # Userdata verification
            rospy.loginfo(f'(DetectFace): Checking userdata..')

            # Do something
            print(userdata)
            try: 
                rospy.loginfo(f'(DetectFace): Starting CV..')
                while self.cap.isOpened():

                    ret, frame = self.cap.read()
                    cv2.imshow("face_detect", frame)


                    res = self.c.detect(frame)
                    if self.log:
                        if res != {}:
                            print(res)
                    if res != {}:
                        rospy.loginfo(f'(DetectFace): CV Detected known person')
                        self.cap.release()
                        cv2.destroyAllWindows()
                        return "out1"
                    # continue
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        self.cap.release()
                        return "out1"

                cv2.destroyAllWindows()
            except:
                self.cap.release()
                return "undo"


            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(ExampleState): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


# # Model Smach States
class RegisterFace(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """

    def __init__(self, 
                 log : bool = True,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        self.log = log
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Setup CV Capture

        self.cap = cv2.VideoCapture(list_available_cam(10))
        self.cap.set(4, 480)
        self.cap.set(3, 640)

        # Setup socket
        host = socket.gethostname()
        port = 12304
        self.c = CustomSocket(host, port, log=False)
        self.c.clientConnect()




    def execute(self, userdata):
        try:
            userdata.name = "Game"
            # Log the execution stage
            rospy.loginfo(f'(RegisterFace): Executing..')

            # Userdata verification
            rospy.loginfo(f'(RegisterFace): Checking userdata..')

            # Do something
            if userdata.name == "":
                raise Exception("No name given")
            
            print(userdata)
            try: 

                # Log the execution stage
                rospy.loginfo(f'(RegisterFace): Starting CV Capturing..')

                while self.cap.isOpened():

                    ret, frame = self.cap.read()
                    cv2.imshow("RegisterFace", frame)

                    
                    
                    res = self.c.register(frame, userdata.name)
                    if res: 
                        if self.log:
                            print(res)
                        # Log the execution stage
                        rospy.loginfo(f'(RegisterFace: CV Captured for {userdata.name}')
                        return "out1"

                    
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        self.cap.release()
                        return "out1"

                cv2.destroyAllWindows()
            except:
                self.cap.release()
                return "undo"


            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(RegisterFace): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


def main():
    # Initialize the node
    NODE_NAME = "smach_core_cv_face_recognition"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1'])

    # Declear Variables for top-state
    sm.userdata.name = ""

    with sm:
        smach.StateMachine.add('REGISTER_FACE',
                            RegisterFace(),
                            remapping={'name': 'name'},
                            transitions={'out1': 'out1',
                                         'undo': 'out1',}
                                        )
        
        # smach.StateMachine.add('DETECT_FACE',
        #                     DetectFace(),
        #                     remapping={'name': 'name'},
        #                     transitions={'out1': 'out1',
        #                                  'undo': 'out1',}
        #                                 )


    
    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # listen_for_kill_command()

    
if __name__ == '__main__':
    main()




