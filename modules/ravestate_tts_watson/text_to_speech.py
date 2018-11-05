from watson_developer_cloud import TextToSpeechV1
from watson_developer_cloud import WatsonApiException


class Text2Speech:
    """
    Synthesize speech from text
    """
    text_to_speech = None

    def __init__(self, **kwargs):
        self.text_to_speech = TextToSpeechV1(
            username=kwargs["t2s_username"],
            password=kwargs["t2s_password"],
            url=kwargs["t2s_url"]
        )

    def synthesize(self, text, filename):
        try:
            # Invoke a Text to Speech method
            with open(filename, 'wb') as file:
                file.write(
                    self.text_to_speech.synthesize(
                        text,
                        'audio/wav',
                        'en-US_AllisonVoice'
                    ).get_result().content)
        except WatsonApiException as ex:
            print("Method failed with status code " + str(ex.code) + ": " + ex.message)
