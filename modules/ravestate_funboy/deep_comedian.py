import re
import random
from typing import List

from .comedian import Comedian
from ravestate_verbaliser import verbaliser

from reggol import get_logger
logger = get_logger(__name__)

import requests

CONFIG = {
    'SERVER_ADDRESS': 'http://localhost',
    'SERVER_PORT': 5050
}


class DeepComedian(Comedian):

    def __init__(self):
        self.address = f"{CONFIG['SERVER_ADDRESS']}:{CONFIG['SERVER_PORT']}"

    def render(self, type: str, utterance: str = None) -> str:
        logger.info(f"{self.__class__.__name__} | Input: {utterance} | Type: {type}")

        result = ""
        if not server_up(self.address):
            logger.error(
                "\n--------"
                "\nThe server does not seem to be running!"
                "\n--------")
        else:
            types = ", ".join(self._get_tokens(type))
            params = {
                'types': types,
                'utterance': " " if utterance is None else utterance
            }
            response = requests.get(self.address, params=params)
            response_json = response.json()
            logger.info(response_json)
            sample = response_json['response'].strip()
            result = sample

        if result == "":
            result = verbaliser.get_random_phrase("type")

        logger.info(f"{self.__class__.__name__} | Result: {result} | Type: {type}")
        return result

    def _get_tokens(self, type: str) -> List[str]:
        size = "medium" if random.random() < 0.4 else "short"
        return [size, type]


def server_up(server_address):
    try:
        status = requests.head(server_address).status_code
    except requests.exceptions.RequestException or requests.exceptions.ConnectionError:
        status = None
    return status == 200