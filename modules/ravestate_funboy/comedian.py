from abc import ABC
from abc import abstractmethod
from typing import List


class Comedian(ABC):
    @abstractmethod
    def render(self, type: List, utterance: str = None) -> str:
        return NotImplemented
