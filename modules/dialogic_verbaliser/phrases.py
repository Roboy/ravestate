from dialogic_util.random_tuple import RandomTuple


class Phrases:
    # get from config
    connection_path = "../dialogic_resources/phrase_lists/segue-connecting-phrases.txt"
    reenter_phrases_path = "../dialogic_resources/phrase_lists/question-answering-reentering-phrases.txt"
    start_phrases_path = "../dialogic_resources/phrase_lists/question-answering-starting-phrases.txt"
    answer_avoiding_path = "../dialogic_resources/phrase_lists/segue-avoid-answer.txt"
    distraction_path = "../dialogic_resources/phrase_lists/segue-distract.txt"
    flattery_path = "../dialogic_resources/phrase_lists/segue-flattery.txt"
    jobs_path = "../dialogic_resources/phrase_lists/segue-jobs.txt"
    pickingup_path = "../dialogic_resources/phrase_lists/segue-pickup.txt"
    profanity_check_path = "../dialogic_resources/phrase_lists/profanity-check-list.txt"
    filter_phrases_path = "../dialogic_resources/phrase_lists/offer-snapchat-filter.txt"
    facts_phrases_path = "../dialogic_resources/phrase_lists/facts.txt"
    roboy_intent_phrases_path = "../dialogic_resources/phrase_lists/info-roboy-intent.txt"
    jokes_phrases_path = "../dialogic_resources/phrase_lists/jokes.txt"
    negative_sentiment_phrases_path = "../dialogic_resources/phrase_lists/negative-sentiment.txt"
    offer_facts_phrases_path = "../dialogic_resources/phrase_lists/offer-facts.txt"
    offer_famous_entities_phrases_path = "../dialogic_resources/phrase_lists/offer-famous-entities.txt"
    offer_jokes_phrases_path = "../dialogic_resources/phrase_lists/offer-jokes.txt"
    offer_math_phrases_path = "../dialogic_resources/phrase_lists/offer-math.txt"
    parser_error_phrases_path = "../dialogic_resources/phrase_lists/parser-error.txt"

    @property
    def segue_connection(self):
        return RandomTuple(self.phrase_reader(self.connection_path))

    @property
    def answering_reenter_phrases(self):
        return RandomTuple(self.phrase_reader(self.reenter_phrases_path))

    @property
    def answering_start_phrases(self):
        return RandomTuple(self.phrase_reader(self.start_phrases_path))

    @property
    def segue_answer_avoiding(self):
        return RandomTuple(self.phrase_reader(self.answer_avoiding_path))

    @property
    def segue_distraction(self):
        return RandomTuple(self.phrase_reader(self.distraction_path))

    @property
    def segue_flattery(self):
        return RandomTuple(self.phrase_reader(self.flattery_path))

    @property
    def segue_jobs(self):
        return RandomTuple(self.phrase_reader(self.jobs_path))

    @property
    def segue_pickingup(self):
        return RandomTuple(self.phrase_reader(self.pickingup_path))

    @property
    def profanity_check_wordlist(self):
        return RandomTuple(self.phrase_reader(self.profanity_check_path))

    @property
    def offer_filter_phrases(self):
        return RandomTuple(self.phrase_reader(self.filter_phrases_path))

    @property
    def facts_phrases(self):
        return RandomTuple(self.phrase_reader(self.facts_phrases_path))

    @property
    def roboy_intent_phrases(self):
        return RandomTuple(self.phrase_reader(self.roboy_intent_phrases_path))

    @property
    def jokes_phrases(self):
        return RandomTuple(self.phrase_reader(self.jokes_phrases_path))

    @property
    def negative_sentiment_phrases(self):
        return RandomTuple(self.phrase_reader(self.negative_sentiment_phrases_path))

    @property
    def offer_facts_phrases(self):
        return RandomTuple(self.phrase_reader(self.offer_facts_phrases_path))

    @property
    def offer_famous_entities_phrases(self):
        return RandomTuple(self.phrase_reader(self.offer_famous_entities_phrases_path))

    @property
    def offer_jokes_phrases(self):
        return RandomTuple(self.phrase_reader(self.offer_jokes_phrases_path))

    @property
    def offer_math_phrases(self):
        return RandomTuple(self.phrase_reader(self.offer_math_phrases_path))

    @property
    def parser_error_phrases(self):
        return RandomTuple(self.phrase_reader(self.parser_error_phrases_path))

    @staticmethod
    def phrase_reader(path):
        with open(path, 'r') as phrases:
            return phrases.readlines()