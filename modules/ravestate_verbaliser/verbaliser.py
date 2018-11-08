import logging
import os
import yaml
from typing import Dict, List

from ravestate_util.random_list import RandomList
from ravestate_verbaliser.qa_phrases import QAPhrases

"""
Produces actual utterances. This should in the future lead to diversifying
the ways Roboy is expressing information.
"""

TYPE_PHRASES: str = "phrases"
TYPE_QA: str = "qa"
phrases: Dict[str, List[str]] = dict()
qa: Dict[str, QAPhrases] = dict()


def add_folder(dirpath: str):
    try:
        for filename in os.listdir(dirpath):
            add_file(os.path.join(dirpath, filename))
    except FileNotFoundError:
        logging.error('Cannot add folder ' + dirpath + ' because it does not exist')


def add_file(path: str):
    try:
        with open(path, 'r') as input_file:
            file_data = list(yaml.safe_load_all(input_file))

        for entry in file_data:
            if entry['type'] == TYPE_PHRASES:
                if str(entry['name']) in phrases:
                    logging.error('Import of ' + path + ' would overwrite phrases-list entry ' + str(entry['name']))
                    return
                phrases[str(entry['name'])] = entry['opts']
            elif entry['type'] == TYPE_QA:
                if str(entry['name']) in qa:
                    logging.error('Import of ' + path + ' would overwrite qa-list entry ' + str(entry['name']))
                    return
                qa[str(entry['name'])] = QAPhrases(entry)
    except FileNotFoundError:
        logging.error('Cannot add file ' + path + ' because it does not exist')


# TODO catch KeyNotFound?
def get_phrase_list(intent: str):
    return RandomList(phrases[intent])


def get_random_phrase(intent: str) -> str:
    return RandomList(phrases[intent]).get_random()


def get_question_list(intent: str):
    return RandomList(qa[intent].questions)


def get_random_question(intent: str) -> str:
    return RandomList(qa[intent].questions).get_random()


def get_successful_answer_list(intent: str):
    return RandomList(qa[intent].successful_answers)


def get_random_successful_answer(intent: str) -> str:
    return RandomList(qa[intent].successful_answers).get_random()


def get_failure_answer_list(intent: str):
    return RandomList(qa[intent].failure_answers)


def get_random_failure_answer(intent: str) -> str:
    return RandomList(qa[intent].failure_answers).get_random()


def get_followup_question_list(intent: str):
    return RandomList(qa[intent].followup_questions)


def get_random_followup_question(intent: str) -> str:
    return RandomList(qa[intent].followup_questions).get_random()


def get_followup_answer_list(intent: str):
    return RandomList(qa[intent].followup_answers)


def get_random_followup_answer(intent: str) -> str:
    return RandomList(qa[intent].followup_answers).get_random()
