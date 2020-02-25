from typing import List

from .comedian import Comedian
from ravestate_verbaliser import verbaliser

from reggol import get_logger
logger = get_logger(__name__)


class SimpleComedian(Comedian):

    def __init__(self):
        self.model = verbaliser

    def render(self, types: List, utterance: str = None) -> str:
        logger.info(f"{self.__class__.__name__} | Input: {input} | Types: {' '.join(types)}")
        result = self.model.get_random_phrase("gpt2-jokes")
        logger.info(f"{self.__class__.__name__} | Input: {input} | Types: {' '.join(types)}")
        return result
