import requests

# put in config
NEW_SESSION_URL = "https://srv11.akinator.com:9152/ws/new_session?callback=&partner=&player=website-desktop&uid_ext_session=&frontaddr=NDYuMTA1LjExMC40NQ==&constraint=ETAT<>'AV'"
ANSWER_URL = "https://srv11.akinator.com:9152/ws/answer"
GET_GUESS_URL = "https://srv11.akinator.com:9152/ws/list"
CHOICE_URL = "https://srv11.akinator.com:9152/ws/choice"
EXCLUSION_URL = "https://srv11.akinator.com:9152/ws/exclusion"


class Api:
    """
    Takes care of the fancy nancy akinator business
    """
    def __init__(self):
        self.data = requests.get(NEW_SESSION_URL).json()
        self.session = self.data['parameters']['identification']['session']
        self.signature = self.data['parameters']['identification']['signature']
        self.guess_data = None

    # get first question
    def get_parameter(self, parameter_type: str) -> str:
        """
        Get method for the get request parameters
        parameter types are: 'step', 'question', 'progression'
        the first call dictionary has an additional nested dict under the key:'step_information' in which the
         parameters are found. The following calls do not have that.
        """
        if 'step_information' in self.data['parameters']:
            return self.data['parameters']['step_information'][parameter_type]
        else:
            return self.data['parameters'][parameter_type]

    def response_get_request(self, response: str):
        """
        Get request for the next question with the response to the previous question in the parameters
        """
        params = {
            "session": self.session,
            "signature": self.signature,
            "step": self.get_parameter('step'),
            "answer": response
        }
        self.data = requests.get(ANSWER_URL, params=params).json()

    def guess_get_request(self) -> dict:
        """
        Get request for the guess
        """
        params = {
            "session": self.session,
            "signature": self.signature,
            "step": self.get_parameter('step')
        }
        self.guess_data = requests.get(GET_GUESS_URL, params=params).json()
        guess = {"name": self.guess_data['parameters']['elements'][0]['element']['name'],
                 "desc": self.guess_data['parameters']['elements'][0]['element']['description']
        }
        return guess

    def choice_get_request(self):
        """
        Get request which is triggered if the guess was right
        """
        params = {
            "session": self.session,
            "signature": self.signature,
            "step": self.get_parameter('step'),
            "element": self.guess_data['parameters']['elements'][0]['element']['id']
        }
        requests.get(CHOICE_URL, params=params)

    def exclusion_get_request(self):
        """
        Get request which is triggered if the guess was wrong
        """
        params = {
            "session": self.session,
            "signature": self.signature,
            "step": self.get_parameter('step'),
            "forward_answer": self.answer_to_int_str("no")
        }
        requests.get(EXCLUSION_URL, params=params)

    def answer_to_int_str(self, answer: str):
        """
        String answer to an integer string which can be processed by akinator api
        """
        if answer == "yes":
            return "0"
        elif answer == "no":
            return "1"
        elif answer == "idk":
            return "2"
        elif answer == "p":
            return "3"
        elif answer == "pn":
            return "4"
        else:
            return "-1"

