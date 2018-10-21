from dialogic_util.random_tuple import RandomTuple
from dialogic_verbaliser.phrases import Phrases
from modules.dialogic_verbaliser.qa_parser import QAParser


class Verbaliser:
    """
    Produces actual utterances. This should in the future lead to diversifying
    the ways Roboy is expressing information.
    """
    path_to_qalist = "../dialogic_resources/sentences/QAList.json"  # get from config
    path_to_info_list = "../dialogic_resources/sentences/RoboyInfoList.json"  # get from config
    qa_list_parser = None
    info_list_parser = None
    phrases = Phrases()

    def __init__(self):
        self.qa_list_parser = QAParser(self.path_to_qalist)
        self.info_list_parser = QAParser(self.path_to_info_list)

    @property
    def greetings(self):
        return RandomTuple(("hello","hi","greetings", "howdy", "hey", "what's up", "greeting to everyone here",
                            "hi there people", "hello world","gruse gott","wazup wazup wazup","howdy humans",
                            "hey hey hey you there"))

    @property
    def roboy_names(self):
        return RandomTuple(("roboi", "robot", "boy", "roboboy", "robot", "roboy"))

    @property
    def consent(self):
        return RandomTuple(("yes", "I do", "sure", "of course", " go ahead"))

    @property
    def denial(self):
        return RandomTuple(("no", "nope", "later", "other time", "not"))

    @property
    def triggers(self):
        return RandomTuple(("talk", "fun", "conversation", "new", "chat"))

    @property
    def farewells(self):
        return RandomTuple(("ciao", "goodbye", "cheerio", "bye", "see you", "farewell", "bye-bye"))

    @property
    def segues(self):
        return RandomTuple(("talking about ","since you mentioned ","on the topic of "))

    @property
    def facts_intro(self):
        return RandomTuple(("did you know ", "did you know that ", "i read that ", "i heard that ",
                            "have you heard this: "))

    @property
    def unclear(self):
        return RandomTuple(("Oh no, I didn't get what you said. ", "I didn't understand you correctly. ",
                            "Sorry? What did you say? "))

    @property
    def dead_ros(self):
        return RandomTuple(("Oh no, where is my ROS connection? I need it. ",
                            "I was looking for my ROS master everywhere but I can find it. ",
                            "I think I have no ROS connection. ",
                            "Hello? Hello? Any ROS master out there? Hmm, I can't hear anybody. "))

    def greet(self):
        return self.greetings.get_random()

    def bye(self):
        return self.farewells.get_random()

    def segue(self):
        return self.segues.get_random() + "interpretation.getAssociation()"

    def fact(self):
        return self.facts_intro.get_random() + "interpretation.getSentence()"




