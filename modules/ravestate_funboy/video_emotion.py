from .emotion import Emotion


class VideoEmotion(Emotion):

    def __init__(self):
        self.model = ""
        self.stream = ""
        self.filter = ""
        self.interval = 5

    def get(self):
        return {'anger': 0.1,
                'fear': 0.1,
                'calm': 0.1,
                'sadness': 0.1,
                'happiness': 0.9,
                'surprise': 0.1,
                'disgust': 0.1}

    def is_positive(self) -> bool:
        return True
