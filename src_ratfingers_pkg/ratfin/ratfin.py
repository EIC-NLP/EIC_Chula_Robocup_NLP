# Version 1.0 - 2023 January 19
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

def clear():
    system = platform.system()
    if platform == 'Windows': # for Windows
        os.system("cls")
    else: # for Unix (MacOS, Linux)
        os.system("clear")

def printclr(text="enter text", color=None):
    if not color:
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

    print("\033[1;{}m{}\033[0m".format(colors[color], "Hello, World!"))

