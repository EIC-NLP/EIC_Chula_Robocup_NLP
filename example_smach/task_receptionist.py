# first run the server, main.py

from nlp_client import *

class Person:
    def __init__(self, name="", drink=""):
        self.name = name
        self.drink = drink
        # add kargs for other characteristicsx



    def __str__(self):
        return self.name + " " + self.drink
    
    def reception_speak(self):
        speak(f"Hello {self.name}, your favourite drink is {self.drink}")

#Door opens
# Person enters
person1 = Person()

# Robots says "Hello, what is your name?"
speak("Hello, what is your name?")

# Person says "My name is <name1>"
person1.name = listen().people

# Robot says "alright <name1>, please stand still while I scan you"
speak("alright " + person1.name + ", please stand still while I scan you")

# Background: Robot scans person & store characteristics

# Robot says "What is your favourite drink?"
speak("What is your favourite drink?")

# Person says "My favourite drink is <drink1>"
person1.drink = listen().object

# Robot says "Thank you <name1>, please take a seat anywhere on the couch"
speak("Thank you " + person1.name + ", please take a seat anywhere on the couch")

# Person sits down
person1.reception_speak()

# Second Person enters
# Robots says "Hello, what is your name?"
# Person says "My name is <name2>"
# Robot says "alright <name2>, please stand still while I scan you"
# Background: Robot scans person & store 
# Robot says "What is your favourite drink?"
# Robot says "Thank you <name2>, please take a seat next to <name1>"
# Person sits down
