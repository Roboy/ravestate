from .emotion import Emotion
from datetime import datetime

import rospy
from rosemo.msg import EmotionResult

from .emotion_queue import EmotionQueue


class VideoEmotion(Emotion):

    def __init__(self, period = 10, frequency = 2):
        self.period = period
        self.frequency = frequency
        self.datapoints = EmotionQueue(self.period * self.frequency)
        self.sub = rospy.Subscriber("rosemo/video", EmotionResult, self.callback)

    def callback(self, data):
        self.datapoints.enqueue(dict(zip(data.labels, data.scores)))

    def get(self):
        return self.datapoints.to_list()

    def __repr__(self):
        return str(self)

    def __str__(self):
        s = f"VideoEmotion: {str(datetime.now())} \n"
        s += 40 * "=" + "\n"
        l = self.get()
        for i in range(0, len(l)):
            s += f"{str(i)}:: {str(l[i])} \n"
        s += 40 * "=" + "\n"
        return s
