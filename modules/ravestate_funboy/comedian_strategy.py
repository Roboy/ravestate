import random as rand
from typing import List

from .simple_comedian import SimpleComedian
from .deep_comedian import DeepComedian

from reggol import get_logger
logger = get_logger(__name__)


class ComedianStrategy:
    comedian = None

    def __init__(self, threshold: float = 0.5):
        self.deep_comedian = DeepComedian()
        self.simple_comedian = SimpleComedian()
        self.count = 0
        self.threshold = threshold

    def configure(self, random: bool = False, interleaving: bool = False):
        self.random = random
        self.interleaving = interleaving
        if self.interleaving:
            self.count += 1
            logger.info(f"Interleaving. Count: {self.count}")
            if self.count % 2 == 0:
                logger.info(f"Choosing SimpleComedian")
                self.comedian = self.simple_comedian
            else:
                logger.info(f"Choosing DeepComedian")
                self.comedian = self.deep_comedian
        elif self.random:
            sample = rand.random()
            logger.info(f"Random. Sampled: {sample}")
            if self.threshold < sample:
                logger.info(f"Choosing SimpleComedian")
                self.comedian = self.simple_comedian
            else:
                logger.info(f"Choosing DeepComedian")
                self.comedian = self.deep_comedian
        else:
            logger.info(f"Default")
            logger.info(f"Choosing DeepComedian")
            self.comedian = self.deep_comedian

    def render(self, type, utterance: str) -> str:
        if self.comedian is not None:
            return self.comedian.render(type, utterance)
        else:
            logger.error("No comedian specified!")
            return ""
