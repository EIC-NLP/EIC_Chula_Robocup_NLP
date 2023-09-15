import os
import platform
import yaml
from ratfin import * 

def get_socket_name():
    
    if platform.system() == "Darwin":
        platform_ = "MacOS"
    else:
        platform_ = platform.system()
    return f"socketconfig_{platform_}_{os.getlogin( )}.yaml"

# SOCKETFILE EXPORT

# The path you want to substitute in
socket_file = "setup/socketconfig_template.yaml"
path = os.path.dirname(os.path.realpath(__file__))
# Load the YAML file
with open(socket_file, "r") as file:
    content = file.read()

# Replace the placeholders with the folder path
replaced_content = content.replace("{}", path)


# Save the updated YAML to a new file in the current directory
with open(get_socket_name(), "w") as file:
    file.write(replaced_content)


# CONFIG FILE EXPORT
config_file = "setup/config_template.py"

# write and rename
with open(config_file, "r") as file:
    content = file.read()
    with open("config.py", "w") as file2:
        file2.write(content)

printclr("init successfull","green")