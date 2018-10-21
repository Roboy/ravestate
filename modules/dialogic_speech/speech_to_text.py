import json

from watson_developer_cloud import SpeechToTextV1
from watson_developer_cloud import WatsonApiException
from os.path import join, dirname


class Speech2Text:
    """
    Recognise speech into text
    """
    speech_to_text = None

    def __init__(self, username, password, url):
        self.speech_to_text = SpeechToTextV1(
            username=username,
            password=password,
            url=url
        )

    def recognise(self, audio_files):
        try:
            # Invoke a Speech to Text method
            for file in audio_files:
                with open(join(dirname(__file__), './.', file),
                          'rb') as audio_file:
                    speech_recognition_results = self.speech_to_text.recognize(
                        audio=audio_file,
                        content_type='audio/wav',
                        timestamps=True,
                        # word_alternatives_threshold=0.9,
                        # keywords=['colorado', 'tornado', 'tornadoes'],
                        # keywords_threshold=0.5
                    ).get_result()
                return self.extract_transcript(json.dumps(speech_recognition_results, indent=2))
        except WatsonApiException as ex:
            print("Method failed with status code " + str(ex.code) + ": " + ex.message)

    @staticmethod
    def extract_transcript(json_response):
        return json_response['results']['alternatives']['transcript']
