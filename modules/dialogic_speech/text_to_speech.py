from watson_developer_cloud import TextToSpeechV1
from watson_developer_cloud import WatsonApiException


class Text2Speech:
    """
    Synthesize speech from text
    """
    text_to_speech = None

    def __init__(self, username, password, url):
        self.text_to_speech = TextToSpeechV1(
            username=username,
            password=password,
            url=url
        )

    def synthesize(self, text):
        try:
            # Invoke a Text to Speech method
            with open('hello_world.wav', 'wb') as audio_file:
                audio_file.write(
                    self.text_to_speech.synthesize(
                        text,
                        'audio/wav',
                        'en-US_AllisonVoice'
                    ).get_result().content)
                return audio_file
        except WatsonApiException as ex:
            print("Method failed with status code " + str(ex.code) + ": " + ex.message)
