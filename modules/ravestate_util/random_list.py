import random


class RandomList(list):
    """
    Extend the builtin list to return a random element
    """
    def get_random(self):
        return super(RandomList, self).__getitem__(random.randrange(len(self)))