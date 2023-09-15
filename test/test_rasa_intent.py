link = "http://localhost:5005/webhooks/rest/webhook"
# code that will post a json to a server and print out the response json
texts = []


# text = "place the coke on the cabinet"
# text = "place the coke on the cabinet"
# text = "get the milk in the kitchen and go to the living room"
# text = "grab me a coke"
text = input()
text = "Navigate to the coat rack, meet Angel, and follow her."
# text = input()
# list_of_text = text.split(" ")
texts.append(text)

# with open('gpsr_prompts_1.txt', 'r') as f:
#     texts = f.read().splitlines()
# with open('gpsr_prompts_1.txt', 'r') as f:
#     texts = f.read().splitlines()

output = []
import requests
# for text in texts:
r = requests.post(url=link,
                    json={
                        "sender": "bot",
                        "message": text
                    })

response = r.json()

print(response)
print()
output.append(response)

# response = r.json()[0]["text"]
# # convert to ast
# import ast
# response = ast.literal_eval(response)
# # indent the ast response
# import json
# print(json.dumps(response, indent=4))
# print(response)