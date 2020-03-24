import copy


class EmotionQueue:

    def __init__(self, size):
        self.size = size
        self.step = 1
        self.offset = self.size - self.step
        self.length = 0
        self.queue = [None for i in range(self.size)]

    def enqueue(self, o):
        if self.length < self.size:
            self.queue[self.length] = o
            self.length += 1
        else:
            self.queue = self.queue[-self.offset:]
            self.queue.append(o)

    def dequeue(self):
        o = self.queue[0]
        self.queue = self.queue[-self.offset:]
        self.queue.append(None)
        self.length -= 1
        return o

    def to_list(self):
        l = copy.deepcopy(self.queue)
        l = [x for x in l if l is not None]
