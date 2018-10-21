import random


class RandomTuple(tuple):
    """
    Extend the builtin tuple to return a random element
    """
    def get_random(self):
        return super(RandomTuple, self).__getitem__(random.randrange(len(self)))