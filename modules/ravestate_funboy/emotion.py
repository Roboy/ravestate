from abc import ABC
from abc import abstractmethod


class Emotion(ABC):
    @abstractmethod
    def get(self):
        return NotImplemented
