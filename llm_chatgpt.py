
import openai
import os
from termcolor import colored
class bcolors:
    RED_FAIL       = '\033[91m'
    GRAY_OK        = '\033[90m'
    GREEN_OK       = '\033[92m'
    YELLOW_WARNING = '\033[93m'
    BLUE_OK        = '\033[94m'
    MAGENTA_OK     = '\033[95m'
    CYAN_OK        = '\033[96m'
    ENDC           = '\033[0m'
    BOLD           = '\033[1m'
    ITALIC         = '\033[3m'
    UNDERLINE      = '\033[4m'
gpt3_5_turbo_key = os.getenv('gpt3_5_turbo_key')

print(f'Your API key is: {gpt3_5_turbo_key}')

# Task specific state
class ChatGPTQuery():
    def __init__(self):
        # String of
        self.messages = [{
            "role": "system",
            "content" : """You’re a kind helpful assistant robot,
                            respond back to me what my commands were but rephrase it like a assistant would
                            by accepting my request. don't ask a question back just do as I says. For example,
                            if I ask you to retrieve a coke. You should respond with something like "Certainly, grabing you a coke now" but make the sentence dynamic dont actually use the word certainly its too formal.
                            This is a role-play"""}]
        openai.api_key = gpt3_5_turbo_key



    def execute(self, prompt, userdata):

        # Log the execution stage
        print(bcolors.BLUE_OK + f"(ChatGPTQuery): Executing..'" + bcolors.ENDC)
        print(bcolors.BLUE_OK + f"(ChatGPTQuery): Executing..'" + bcolors.ENDC)

        # Copy the assistant mode prompt
        messages = self.messages.copy()


        # Log the execution stage
        print(bcolors.BLUE_OK + f'(ChatGPTQuery): Prompt: {prompt}' + bcolors.ENDC)

        # Append to messages
        messages.append({"role": "user", "content": prompt})

        # packaged the messages and post request
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=messages
        )
        # extract the content of the post request
        chat_response = completion.choices[0].message.content

        # Save the response to userdata
        userdata.chatgpt_response = chat_response
        return userdata

