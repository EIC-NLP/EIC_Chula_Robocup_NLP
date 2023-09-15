link = "http://localhost:5005/model/parse"
# code that will post a json to a server and print out the response json



text = "can you find Simone from the bedroom and escort her back?"
import requests
r = requests.post(url=link,
                    json={"text": text})


response = r.json()
# json indent 
import json
print(json.dumps(response, indent=4))
# print(response)