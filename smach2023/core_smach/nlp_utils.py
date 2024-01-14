
# run with conda env: nlp
import roslib
import rospy
import smach
import smach_ros
from nlp_client import *
from termcolor import colored


def prompt_user_repeat():
    speak("Sorry I didn't get that. Please rephrase that?")

# define state speak


class WakeWord(smach.State):
    """
    return "out1" if wakeword is detected
    >>> outcome map: {'WAKEWORD_DETECTED': 'out1'}

       """
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('(WakeWord): Listening for Hey Walkie')
        ww_listen()
        rospy.loginfo('WakeWord detected')
        return "out1"


class Speak(smach.State):
    def __init__(self, text, keys=None, response_debug=False):
        if keys is None:
            keys = []

        smach.State.__init__(self, outcomes=['out1', 'out0'], input_keys=keys)

        self.response_debug = response_debug
        self.text = text
        self.keys = keys

    def execute(self, userdata):
        try:
            rospy.loginfo(f'(Speak): Executing..')

            # Prepare arguments for the format string from userdata
            args = [getattr(userdata, key)
                    for key in self.keys] if self.keys else []

            # Prepare the text to speak
            text = self.text.format(*args)

            rospy.loginfo(f'Speaking : {text}')

            # speak the intent
            speak(text)

            return "out1"
        except Exception as e:
            print(e, "red")
            return "out0"


class GetEntities(smach.State):
    """
    smach.StateMachine.add('GET_ENTITIES',
                           GetEntities(intent=True, name=True, object=False, location=False,
                                                       speak_debug=speak_debug,
                                                       response_debug=response_debug,
                                                       timeout=2),
                           transitions={'out1': 'NEXT_STATE',
                                        'out0': 'END'},
                           remapping={'listen_intent': 'intent',
                                      'listen_name': 'name',
                                      'listen_object': 'object',
                                      'listen_location': 'location'})
    """

    def __init__(self,
                 intent: bool = False,
                 name: bool = False,
                 object: bool = False,
                 location: bool = False,
                 confidence: bool = False,
                 speak_debug: bool = False,
                 response_debug: bool = False,
                 timeout=0):

        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(intent, bool):
            raise ValueError("Argument 'intent' must be of type bool")
        if not isinstance(name, bool):
            raise ValueError("Argument 'name' must be of type bool")
        if not isinstance(object, bool):
            raise ValueError("Argument 'object' must be of type bool")
        if not isinstance(location, bool):
            raise ValueError("Argument 'location' must be of type bool")
        if not isinstance(confidence, bool):
            raise ValueError("Argument 'confidence' must be of type bool")
        if not isinstance(speak_debug, bool):
            raise ValueError("Argument 'speak_debug' must be of type bool")
        if not isinstance(response_debug, bool):
            raise ValueError("Argument 'response_debug' must be of type bool")
        if not isinstance(timeout, int):
            raise ValueError("Argument 'timeout' must be of type integer")

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.valid_out = False  # check if the output is valid

        # adding enities to extract
        self.attributes = []

        if intent:
            self.attributes.append('intent')
        if name:
            self.attributes.append('name')
        if object:
            self.attributes.append('object')
        if location:
            self.attributes.append('location')

        # timout config
        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_' +
                                         a for a in self.attributes],
                             output_keys=['listen_' + a for a in self.attributes])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetEntities): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetEntities): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # Check the object & confidence for each attribute
                for a in self.attributes:
                    attr_value = getattr(res_obj, a, "")
                    if attr_value != "" and res_obj.confidence > min_confidence:
                        setattr(userdata, 'listen_'+a, attr_value)
                        self.valid_out = True

                # If any valid attribute is found, break
                if self.valid_out:
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetEntities): No attribute detected in the timeout period")
                raise Exception(
                    "(GetEntities): No attribute detected in the timeout period")

            return "out1"

        except Exception as e:
            print(e, "red")
            return "out0"


# define state GetIntent
class GetIntent(smach.State):
    """
    smach.StateMachine.add('GET_INTENT',
                               GetIntent(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_NAME',
                                            'out0': 'END'},
                               remapping={'listen_intent': 'intent'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_intent', 'listen_text'],
                             output_keys=['listen_intent', 'listen_text'])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetIntent): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetIntent): [{self.listen_counter}]Listening..')

                # listen for user
                # res_obj = listen(intent=True)
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.intent != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

                # Ask the user to repeat if not valid
                prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetIntent): No intent detected in the timeout period")
                raise Exception(
                    "(GetIntent): No intent detected in the timeout period")

            # TODO check if object is in the database
            # TODO if not, ask for the object
            # TODO check if it's what the user wants, check if null

            # Store intent in userdata for later use
            userdata.listen_intent = res_obj.intent

            # Store text in userdata for later use
            userdata.listen_text = res_obj.text

            # Log the intent
            rospy.loginfo(f'(GetIntent): {userdata.listen_intent}')

            # speak the intent if debug
            if self.speak_debug:
                speak(f'(GetIntent): {userdata.listen_intent}')

            return "out1"
        except Exception as e:
            print(e, "red")
            return "out0"


class GetName(smach.State):
    """
    smach.StateMachine.add('GET_NAME',
                               GetName(speak_debug=speak_debug,
                                       response_debug=response_debug,
                                       timeout=2),
                               transitions={'out1': 'GET_OBJECT',
                                            'out0': 'END'},
                               remapping={'listen_name': 'name'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.counter = 0
        self.timeout = timeout
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             output_keys=["listen_name"])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetName): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(f'(GetName): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.people != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetName): No name detected in the timeout period")
                raise Exception(
                    "(GetName): No name detected in the timeout period")

            # Store intent in userdata for later use
            name = res_obj.people
            userdata.listen_name = res_obj.people

            # Log the name
            rospy.loginfo(f'(GetName): {name}')

            # speak the name if debug
            if self.speak_debug:
                speak(f'(GetName): {name}')

            return "out1"
        except Exception as e:
            print(e, "red")
            return "out0"


class GetObject(smach.State):
    """
    smach.StateMachine.add('GET_OBJECT',
                               GetObject(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_LOCATION',
                                            'out0': 'END'},
                               remapping={'listen_object': 'object'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_object'],
                             output_keys=["listen_object"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetObject): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetObject): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.object != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetObject): No object detected in the timeout period")
                raise Exception(
                    "(GetObject): No object detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_object = res_obj.object

            # Log the name
            rospy.loginfo(f'(GetObject): {userdata.listen_object}')

            # speak the object if debug
            if self.speak_debug:
                speak(f'(GetObject): {userdata.listen_object}')

            return "out1"
        except Exception as e:
            print(e, "red")
            return "out0"


class GetLocation(smach.State):
    """
    smach.StateMachine.add('GET_LOCATION',
                               GetLocation(speak_debug=speak_debug,
                                           response_debug=response_debug,
                                           timeout=2),
                               transitions={'out1': 'END',
                                            'out0': 'END'},
                               remapping={'listen_location': 'location'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_location'],
                             output_keys=["listen_location"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetLocation): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetLocation): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.room != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetLocation): No location detected in the timeout period")
                raise Exception(
                    "(GetLocation): No location detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_location = res_obj.room

            # Log the name
            rospy.loginfo(f'(GetLocation): {userdata.listen_location}')

            # speak the location if debug
            if self.speak_debug:
                speak(f'(GetLocation): {userdata.listen_location}')

            return "out1"
        except Exception as e:
            print(e, "red")
            return "out0"


def main():
    speak_debug = False
    response_debug = False

    rospy.init_node('utils_nlp')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.intent = ""
    sm.userdata.name = ""
    sm.userdata.object = ""
    sm.userdata.location = ""

    # open the container
    with sm:
        smach.StateMachine.add('GET_ENTITIES',
                               GetEntities(intent=True,
                                           object=True,
                                           speak_debug=speak_debug,
                                           response_debug=response_debug,
                                           timeout=2),
                               transitions={'out1': 'END',
                                            'out0': 'END'},
                               remapping={'listen_intent': 'intent',
                                          'listen_name': 'name',
                                          'listen_object': 'object',
                                          'listen_location': 'location'})
        # smach.StateMachine.add('WAKEWORD',
        #                        WakeWord(),
        #                        transitions={'out1':'SPEAK'})

        # smach.StateMachine.add('SPEAK',
        #                        Speak(text="Hello, what can I do for you?",
        #                              response_debug=response_debug),
        #                        transitions={'out1': 'GET_INTENT',
        #                                     'out0': 'END'})

        # smach.StateMachine.add('GET_INTENT',
        #                        GetIntent(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_NAME',
        #                                     'out0': 'END'},
        #                        remapping={'listen_intent': 'intent'})

        # smach.StateMachine.add('GET_NAME',
        #                        GetName(speak_debug=speak_debug,
        #                                response_debug=response_debug,
        #                                timeout=2),
        #                        transitions={'out1': 'GET_OBJECT',
        #                                     'out0': 'END'},
        #                        remapping={'listen_name': 'name'})

        # smach.StateMachine.add('GET_OBJECT',
        #                        GetObject(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_LOCATION',
        #                                     'out0': 'END'},
        #                        remapping={'listen_object': 'object'})

        # smach.StateMachine.add('GET_LOCATION',
        #                        GetLocation(speak_debug=speak_debug,
        #                                    response_debug=response_debug,
        #                                    timeout=2),
        #                        transitions={'out1': 'END',
        #                                     'out0': 'END'},
        #                        remapping={'listen_location': 'location'})

    # Execute SMACH plan
    outcome = sm.execute()
    print(sm.userdata.__dict__["_data"])


if __name__ == '__main__':
    main()
