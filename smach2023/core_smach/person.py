import rospy
import roslib
import smach_ros
import smach
from nlp_client import * 


class Person():
    def __init__(self, 
                    name: str = None, 
                    favorite_drink: str = None, 
                    age: int = None, 
                    shirt_color: str = None, 
                    hair_color: str = None, 
                    # **kwargs
                    ) -> None:
        self.name = name
        self.favorite_drink = favorite_drink
        self.age = age
        self.shirt_color = shirt_color
        self.hair_color = hair_color
        # Store any additional attributes as instance variables
        # for key, value in kwargs.items():
        #     setattr(self, key, value)

    def __str__(self):
        return f"{self.name} is {self.age} years old and has {self.hair_color} hair."

    def __repr__(self):
        return f"Person(name={self.name}, age={self.age}, shirt_color={self.shirt_color}, hair_color={self.hair_color}, favorite_drink={self.favorite_drink})"
    
    def __dict__(self):
        return self.__dict__
        
def main():

    Game = Person()
    Game.dicksize = 555555
    print(Game.dicksize)
    print(Game.__dict__)
    

if __name__ == "__main__":
    main()