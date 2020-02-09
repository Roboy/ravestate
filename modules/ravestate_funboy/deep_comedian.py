from ravestate_verbaliser import verbaliser


class DeepComedian():

    def __init__(self):
        self.model = ""

    def render(self, input, category):
        return verbaliser.get_random_phrase("gpt2-jokes")
