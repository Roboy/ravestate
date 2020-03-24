from .emotion import Emotion
from datetime import datetime

import rospy
from rosemo.msg import EmotionResult


class AudioEmotion(Emotion):

    def __init__(self):
        self.datapoint = {}
        self.sub = rospy.Subscriber("rosemo/audio", EmotionResult, self.callback)

    def callback(self, data):
        self.datapoint = dict(zip(data.labels, data.scores))

    def get(self):
        return self.datapoint

    def __repr__(self):
        return str(self)

    def __str__(self):
        s = f"AudioEmotion: {str(datetime.now())} \n"
        s += 40 * "=" + "\n"
        s += f"0:: {str(self.datapoint)} \n"
        s += 40 * "=" + "\n"
        return s
