from typing import List

import rospy

from .audio_emotion import AudioEmotion
from .video_emotion import VideoEmotion

from reggol import get_logger
logger = get_logger(__name__)


class EmotionStrategy:
    comedian = None

    def __init__(self, alpha = 1, beta = 0):
        self.video_emotion = None
        self.audio_emotion = None
        self.alpha = alpha
        self.beta = beta
        self.positivity = []
        self.negative = ['anger', 'disgust', 'fear', 'sadness']
        self.positive = ['happiness', 'surprise']

    def configure(self, video: bool = True, audio: bool = False):
        if video:
            if audio:
                logger.info(f"Video & Audio")
                rospy.init_node('videoaudio')
                self.video_emotion = VideoEmotion()
                self.audio_emotion = AudioEmotion()
                rospy.spin()
            else:
                logger.info(f"Only Video")
                self.audio_emotion = None
                rospy.init_node('onlyvideo')
                self.video_emotion = VideoEmotion()
                rospy.spin()
        else:
            if audio:
                logger.info(f"Only Audio")
                self.video_emotion = None
                rospy.init_node('onlyaudio')
                self.audio_emotion = AudioEmotion()
                rospy.spin()
            else:
                logger.info(f"No strategy chosen")
                self.positivity = []

    def get_positivity(self) -> List[int]:

        emotions = []

        if self.video_emotion is not None:
            video_datapoints = self.video_emotion.get()
            if self.audio_emotion is not None:
                audio_datapoints = self.audio_emotion.get()
                emotions = [{x: y.get(x, 0) * self.alpha + audio_datapoints.get(x, 0) * self.beta for x in set(y)}
                            for y in video_datapoints]
            else:
                emotions = video_datapoints
        else:
            if self.audio_emotion is not None:
                emotions = self.audio_emotion.get()

        self.positivity = []
        if len(emotions) > 0:
            for e in emotions:
                negative_sum = sum([e[x] for x in self.negative])
                positive_sum = sum([e[x] for x in self.positive])
                if negative_sum < positive_sum:
                    self.positivity.append(1)
                elif negative_sum > positive_sum:
                    self.positivity.append(-1)
                else:
                    self.positivity.append(0)
        else:
            self.positivity = [0]

        return self.positivity
