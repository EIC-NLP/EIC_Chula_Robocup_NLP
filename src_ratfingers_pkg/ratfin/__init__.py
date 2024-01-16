__version__ = '2.2.0'
"""
Date: 19 Jan 2023
"""

def hello_world():
    print("This is my first pip package!")

import os
import platform

colors = {
    "black": "30",
    "red": "31",
    "green": "32",
    "yellow": "33",
    "blue": "34",
    "magenta": "35",
    "cyan": "36",
    "white": "37"
}

def os.system("clear"):
    system = platform.system()
    if system == 'Windows': # for Windows
        os.system("cls")
    elif system == 'Darwin': # for MacOS
        os.system("clear")
    elif system == 'Linux': # for Linux
        os.system("clear")
    else: # for other platforms
        print('Unknown platform')


def printclr(text="enter text", color : str = ''):
    if (color == '') or (color not in colors):
        print("please enter a color from the following list: ")
        print("""
    black:  30
    red:    31
    green:  32
    yellow: 33
    blue:   34
    magenta:35
    cyan:   36
    white:  37""")
    else:
        print("\033[1;{}m{}\033[0m".format(colors[color], text))

def upDirPath(path="", up=1, filename=""):
    if path == "":
        printclr("missing argument for upDirPath()","red")
        printclr("""enter the following on top of the upDirPath() function:

    path = os.path.dirname(os.path.realpath(__file__))

Your code should look like this:

    path = os.path.dirname(os.path.realpath(__file__))
    print(upDirPath(path, up=1, filename="nlp_config.json"))

 ""","blue")
        return "upDirPath() error"
    else:
        if platform.system() == 'Windows':
            path = path.split("\\")
            # printclr(path,"red")
            path = "\\".join(path[:-up:]) + f"\\{filename}"
            # printclr(path,"red")
            return path

        else:
            path = path.split("/")
            # printclr(path,"red")
            path = "/".join(path[:-up:]) + f"/{filename}"
            # printclr(path,"red")
            return path
