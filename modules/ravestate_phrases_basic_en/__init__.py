
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "en"))

intent_qa_start = "question-answering-starting-phrases"
intent_qa_unsure = "unsure-question-answering-phrases"
intent_avoid_answer = "segue-avoid-answer"
intent_consent = "consent"
intent_distract = "distract"
intent_facts = "facts"
intent_farewells = "farewells"
intent_fillers = "fillers"
intent_flattery = "flattery"
intent_greeting = "greeting"
intent_jokes = "jokes"
intent_offer_fact = "offer-fact"
intent_prompt_qa = "question-answering-prompt"
intent_topic_change = "topicchange"
intent_unclear = "unclear"
