import requests

from reggol import get_logger
logger = get_logger(__name__)


NEW_SESSION_URL = "https://srv11.akinator.com:9152/ws/new_session?callback=&partner=&player=website-desktop&uid_ext_session=&frontaddr=NDYuMTA1LjExMC40NQ==&constraint=ETAT<>'AV'"
ANSWER_URL = "https://srv11.akinator.com:9152/ws/answer"
GET_GUESS_URL = "https://srv11.akinator.com:9152/ws/list"
# CHOICE_URL = "https://srv11.akinator.com:9152/ws/choice"
# EXCLUSION_URL = "https://srv11.akinator.com:9152/ws/exclusion"
# GLB_URL = "https://pastebin.com/gTua3dg2"

akinator_data = None


class Api:
    def __init__(self):
        self.data = requests.get(NEW_SESSION_URL).json()
        self.session = self.data['parameters']['identification']['session']
        self.signature = self.data['parameters']['identification']['signature']
        self.dict_structure = ['parameters', 'step_information']

    def get_session(self):
        return self.session

    def get_signature(self):
        return self.signature

    # get first question
    def get_parameter(self, parameter_type: str) -> dict:
        if 'step_information' in self.data['parameters']:
            return self.data['parameters']['step_information'][parameter_type]
        else:
            return self.data['parameters'][parameter_type]

    def get_progression(self) -> float:
        return float(self.data['parameters']['progression'])

    def response_get_request(self, response: str):
        params = {
            "session": self.get_session(),
            "signature": self.get_signature(),
            "step": self.get_parameter('step'),
            "answer": response
        }
        self.data = requests.get(ANSWER_URL, params=params).json()

    def guess_get_request(self) -> dict:
        params = {
            "session": self.get_session(),
            "signature": self.get_signature(),
            "step": self.get_parameter('step')
        }
        guess_data = requests.get(GET_GUESS_URL, params=params).json()
        guess = {"name": guess_data['parameters']['elements'][0]['element']['name'],
                 "desc":guess_data['parameters']['elements'][0]['element']['description']
        }
        return guess
