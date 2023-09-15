'Author: Game Tinapat Limsila'
import json
class Response:
    """Response for the NLP-pipeline. 
    Possible Entities: [object, furniture, storage, adj_object, people, people_action, place_object, position, demonstrative, rpos, door_action, room]
    
    Text from stt
    Intent from NLU Rasa
    
    Example of how to use Response for NLP Robocup ::

        >>> x = Response()
        >>> print(x)
        Response(
            text=''
            intent=''
            confidence=0.0
            object=''
            people=''
        )
        >>> x.join_json("{
            \"intent\": \"restaurant_order\", 
            \"object\": \"coca-cola\", 
            \"people\": \"me\",
            \"place\": \"your mum\"
            }")
        >>> print(x)
        Response(
            text=''
            intent='restaurant_order'
            confidence=0.0
            object='coca-cola'
            people='me'
            place='your mum'
        )
        >>> x.your_mum = "fat"
        >>> print(x)
        Response(
            text=''
            intent='restaurant_order'
            confidence=0.0
            object='coca-cola'
            people='me'
            place='your mum'
            your_mum = "fat" <--
        )

    """

    def __init__(
        self,
        text: str = "",
        intent: str = "",
        confidence: float = 0.00,
        object: str = "",
        people: str = "",
        room: str = "",

        **kwargs  # To handle future attributes
    ):
        self.text = text
        self.intent = intent
        self.confidence = confidence
        self.object = object
        self.people = people
        self.room = room

        # Store any additional attributes as instance variables
        for key, value in kwargs.items():
            setattr(self, key, value)

    def join_json(self, text: str, verbose = False):
        json_dict = json.loads(str(text))
        print(f"{json_dict=}")
        for key, value in json_dict.items():
            setattr(self, key, value)

    def join_dict(self, in_dict=None, verbose = False):
        # print(in_dict)
        for key, value in in_dict.items():
            setattr(self, key, value)

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def __str__(self):
        attributes = [
            f'{attr}={getattr(self, attr)!r}'
            for attr in vars(self)
            if not callable(getattr(self, attr)) and not attr.startswith("__")
        ]
        output = '\n\t'.join(attributes)
        return f'''Response(\n\t{output}\n)'''

# response = Response(wakeword=False, status_code=200, text='Hello', intent='greet', object='table', people='me')
# response.sexy = "sexy"
# print(response)


if __name__ == "__main__":
    dick = "{\"intent\": \"restaurant_order\", \"object\": \"coca-cola\", \"people\": \"me\",\"place\": \"your mum\"}"
    # print(Response())
    x = Response()
    # y = Response(wakeword=False, status_code=200, text='Hello', intent='greet', object='table', people='me')
    # y.sexy = "sexy"
    y= Response()
    x.join_json()
    
    print(x)
    print(y)
    print(dick)
    print("this is x")
    x.join_json(y.to_json())
    print(x)
    # print(x.to_json())
    # print(Response().join_json(x.to_json()))
    # print(y)
    
