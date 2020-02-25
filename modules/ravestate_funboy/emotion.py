from abc import ABC
from abc import abstractmethod


class Emotion(ABC):
    @abstractmethod
    def get(self):
        return NotImplemented

    @abstractmethod
    def is_positive(self) -> bool:
        return NotImplemented
