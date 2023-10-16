# Robocup-2023-NLP

## Table of Contents

## Table of Contents

- [Robocup-2023-NLP](#robocup-2023-nlp)
  - [Table of Contents](#table-of-contents)
  - [Requires](#requires)
  - 🚀 [How-To Setup](#how-to-setup)
  - [Checking if everything is installed correctly](#checking-if-everything-is-installed-correctly)
  - [Running](#running)
  - [Reference](#reference)
  - [Rasa NLU/NLG README](https://github.com/EIC-NLP/Robocup-2023-NLP/tree/main/rasa)
  - [Automatic Speech Recognition README](https://github.com/EIC-NLP/Robocup-2023-NLP/tree/main/stt)
  - [WakeWord README](https://github.com/EIC-NLP/Robocup-2023-NLP/tree/main/wakeword)
  - [Text-to-Speech README](https://github.com/EIC-NLP/Robocup-2023-NLP/tree/main/tts)

## Requires

- python 3.9.16 in conda named "nlp"
- CUDA 12.1 (if on Ubuntu, if mac not needed)


## How-To Setup

### 1. Setup virtual environments with Conda

Create anaconda virtual environment. (make sure you are in the root directory)

NOTE: if you run into importing problem with VS Code try to use the base version of Anaconda with
```conda create -n "nlp" python=3.9.16```

Activate conda
   if on VS Code, just select the intreperter to 3.9.13 "nlp" conda

Check if nlp is installed with ```conda activate nlp```


### 2. Install Dependencies
To install everything run
```shell
source init.sh
```

NOTE: everything is in the requirements.txt file. and is run by pipinstall.sh file. If you need to use another verison of CUDA change the version in the pipinstall.sh file.

If there's a problem with pyaudio use `conda install pyaudio` also in the requirements.txt file

After running the script, everything should be installed and ready to run.
        If this works then skip to step 5. & if you are running offline then skip to Runnning.

### 2.1 If you want to install everything manually
```shell
pip install -r requirements.txt
```

### 3. (Optional)Setup NLP parameters

#### 3.1 Setup runtime parameters
run `setup_tool.py` to setup the parameters for all the files. This will config the mic for stt & Wakeword


### 4. (Optional)Setup-NLP-Client (Optional, ran by init.sh)
For interfacing with NLP server(by running main), NLP Client is required. This will be install from github or locally from the .

Install Local
```shell
# change diretory to client_src
cd client_src
# install from local
pip install -e .
```

<!-- Install the lastest from Github
```shell
pip install git+https://github.com/EIC-NLP/Robocup-2023-NLP/client_src.git
``` -->

### 5. Configuration for all the keys(Only if running Online)
All the keys are in the `config.py` file. Make sure to change the keys to your own keys.
1. AzureKey for TTS, can be acquired from Azure Cognitive Service
2. AccessKey for Wakeword, get from Porcupine website
3. ChatGPT(Expensive), only few can use it. Get from OpenAI


## Checking if everything is installed correctly
Run `python main.py` and enter 1 (nlpall) to check if all the libraries installed correctly

when running the socket fle, rasa1 will take the longest time to load. This is normal. It will take about 2 minutes to load.
when rasa1 is loaded, it will look like the image below.

**If everything is ok, all the server should be running. Else check the error message and debug.**

it should say `rasa1 | 2023-01-20 21:54:50 INFO     root  - Rasa server is up and running.`
![alt text](misc/runningexample1.png)


## Running

1. Run the server with `python main.py` and enter 1 (nlpall)
2. run the code below

```
from nlp_client import *

# speak with tts
speak()
"""
Output: {
    'synthesised': 'hello there'
    }

# listen with stt
listen()
"""
Output: {
    'body': ' grasp me a Coke.',
    'confidence': 0.8590593338012695,
    'intent': 'restaurant_order',
    'object': 'Coke',
    'recipient_id': 'bot'
    }
"""


# listen with wakeword
ww_listen()
"""
Output: {
    'body': ' Can I get the check please?',
    'confidence': 0.9713435173034668,
    'intent': 'restaurant_checkout',
    'recipient_id': 'bot'}
"""
```

```

## 2. Error handling

Saying `" Hello there, my name is Gabe.",` to rasa will return an error message with a confidence of 0. This is because the intent is not in the training data. This is a good way to test the error handling. :
it will output the following

```{
"body": " Hello there, my name is Gabe.",
"confidence": 0,
"recipient_id": "bot"
}

```

## Testing

**[Insomnia](https://insomnia.rest/download)** is recommended for testing the POST request. files are in the testing folder. This is the endpoint that will be use in the real deployment.

### To test the rasa NLU API without actions

This will only return the **intent and entities**

1. http://localhost:5005/model/parse [POST request]
2. format: {"text": " hello world"}

### To test the rasa NLU API with actions

This will only return the **intent & execute actions (if any)**
Use this to test the actions,

1. http://localhost:5005/webhooks/rest/webhook [POST request]
2. format: `{"sender": "human", "message": "hello world"}`
3. should return `[{"recipient_id": "human","text": "Hello World!"}]` & execute the action

# Reference

- [Rasa docs](https://rasa.com/docs/)
