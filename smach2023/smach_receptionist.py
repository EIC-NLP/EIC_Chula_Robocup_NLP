# run with conda env: nlp
import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from termcolor import colored

from core_smach.person import Person
from utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)

# Task specific state
class AddPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1','out0'],
                            input_keys=['name','favorite_drink','age',
                                         'shirt_color','hair_color',
                                         'people_list', 'people_index'],
                            output_keys=['people_list','people_index'])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(AddPerson): Executing..')
        
        p = Person(name=userdata.name,
                   favorite_drink=userdata.favorite_drink,
                   age=userdata.age,
                   shirt_color=userdata.shirt_color,
                   hair_color=userdata.hair_color
                   )
        
        # Add person object to people_list
        userdata.people_list.append(p)

        # print all people attributes
        # for person in userdata.people_list:
        #     print(person.__dict__)

        userdata.people_index += 1
        rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')
        
        return 'out1'

# Task specific state
class IntroducePeople(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1', 'out0'],
                            input_keys=['people_list', 'people_index'],
                            output_keys=['people_index'])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')

        # Extract people from userdata
        person1 : Person = userdata.people_list[0]
        person2 : Person = userdata.people_list[1]

        # Contruct text to speak
        text : str = f"""Hello, {person1.name}. 
        you are {person1.age} years old 
        has {person1.hair_color} hair 
        wears a {person1.shirt_color} shirt and 
        your favorite drink is {person1.favorite_drink}. 
        Next to you is {person2.name}. 
        they are {person2.age} years old 
        has {person2.hair_color} hair 
        wears a {person2.shirt_color} shirt 
        and their favorite drink is {person2.favorite_drink}."""
        
        # Speak the text
        nlp_client.speak(text=text)

        return 'out1'

# Task specific state
class DummyCv(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             output_keys=['age', 'shirt_color', 'hair_color']
                             )

    def execute(self, userdata):
        userdata.age = 12
        userdata.shirt_color = "blue"
        userdata.hair_color = "negro"

        return 'out1'


# Task specific state
class CheckPeople(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['people_index','max_people'],
                             output_keys=['people_index'])
        
    def execute(self, userdata):
        # Log the execution stage
        rospy.loginfo(f'(CheckPeople): Executing..')

        if userdata.people_index < userdata.max_people:
            # Log the execution stage
            rospy.loginfo(f'(CheckPeople): More people to add, {userdata.people_index} < {userdata.max_people}')
            return "out1"
        else:
            # Log the execution stage
            rospy.loginfo(f'(CheckPeople): Max Capacity')
            return "out0"


def main():
    speak_debug = False
    response_debug = False
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.max_people = 2
    sm.userdata.intent = ""
    sm.userdata.people_list : list[Person] = [] 
    sm.userdata.people_index = 0
    sm.userdata.age = 0
    sm.userdata.hair_color = ""
    sm.userdata.shirt_color = ""
    sm.userdata.name = ""
    sm.userdata.favorite_drink = ""
    sm.userdata.couch_location = [0,1,2]
    

    with sm:

        smach.StateMachine.add('CHECK_PEOPLE',
                            # Speak(text="Please stand still for a moment while I scan you."),
                            CheckPeople(),
                            remapping={'people_index': 'people_index'},
                            transitions={'out1': 'SPEAK_SCAN',
                                        'out0': 'INTRODUCE_PEOPLE'}
                                        )
        
        smach.StateMachine.add('INTRODUCE_PEOPLE',
                            # Speak(text="Please stand still for a moment while I scan you."),
                            IntroducePeople(),
                            remapping={'people_list': 'people_list'},
                            transitions={'out1': 'out0',
                                        'out0': 'out0'}
                                        )
        
        smach.StateMachine.add('SPEAK_SCAN',
                            Speak(text=""""Welcome guest My name is Walkie I will be your receptionist today. 
                            Please stand still for a moment while I scan you."""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'DUMMY_CV',
                                            'out0': 'out0'}
                                        )
        
        smach.StateMachine.add('DUMMY_CV',
                            DummyCv(),
                            transitions={'out1': 'SPEAK_ASK_NAME',
                                            'out0': 'out0'},
                                remapping={'age': 'age', 'shirt_color': 'shirt_color', 'hair_color': 'hair_color'})
        
        smach.StateMachine.add('SPEAK_ASK_NAME',
                                Speak(text="Thank you, I have recoreded your information. What's your name"),
                                # Speak(text="Walkie name"),
                            transitions={'out1': 'GET_NAME',
                                            'out0': 'out0'},)
        
        smach.StateMachine.add('GET_NAME',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=3),
                            transitions={'out1': 'SPEAK_ASK_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT',
                    Speak(text="Hello {}, What's your favorite drink?",
                        # Speak(text="Hello {}, favorite drink?",
                            keys=["name"]),
                            transitions={'out1': 'GET_OBJECT',
                                    'out0': 'out0'},
                            remapping={'name': 'name'})

        smach.StateMachine.add('GET_OBJECT',
                            GetObject(speak_debug=speak_debug,
                                        response_debug=response_debug,
                                        timeout=3),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT',
                                Speak(text="oh!, I like {} too",
                                    keys=["favorite_drink"]),
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'ADD_PERSON',
                                            'out0': 'out0'})
                                                
        smach.StateMachine.add('ADD_PERSON',
                                AddPerson(),
                                transitions={'out1': 'CHECK_PEOPLE', 
                                             'out0': 'out0'},
                                remapping={"age":"age",
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        "people_list":"people_list",
                                        'people_index':'people_index'}
                                )
        
        
    from emerStopDumb import EmergencyStop
    es = EmergencyStop()
    import time
    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    
    import os

    while True:
        
        pid = os.getpid()
        if es.stop_flag:
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


    
if __name__ == '__main__':
    main()
