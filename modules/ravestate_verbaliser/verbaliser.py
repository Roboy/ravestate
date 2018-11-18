import logging
import os
import random
import yaml
from typing import Dict, List, Optional

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
    """
    Parse all files in this folder and add them to the current phrases- and qa-dicts

    :param dirpath: Path to the folder that should be added
    """
    try:
        for filename in os.listdir(dirpath):
            add_file(os.path.join(dirpath, filename))
    except FileNotFoundError:
        logging.error('Cannot add folder ' + dirpath + ' because it does not exist')


def add_file(path: str):
    """
    Parse the file and add the contents to the current phrases- and qa-dicts

    :param path: Path to the file that should be added
    :return: True if the file was successfully added, False if there was an error while adding the file
    """
    try:
        with open(path, 'r') as input_file:
            file_data = list(yaml.safe_load_all(input_file))

        for entry in file_data:
            if entry['type'] == TYPE_PHRASES:
                if str(entry['name']) in phrases:
                    logging.error('Import of section ' + str(entry['name']) + ' in ' + path +
                                  ' would overwrite phrases-list entry ' + str(entry['name']))
                phrases[str(entry['name'])] = entry['opts']
            elif entry['type'] == TYPE_QA:
                if str(entry['name']) in qa:
                    logging.error('Import of section ' + str(entry['name']) + ' in ' + path +
                                  ' would overwrite qa-list entry ' + str(entry['name']))
                qa[str(entry['name'])] = QAPhrases(entry)
            else:
                logging.error('Unknown type given for section ' + str(entry['name']) + ' in ' + path)

        return True
    except (FileNotFoundError, KeyError):
        logging.error('Cannot add file ' + path + ' because it does not exist or is in the wrong format.')
        return False


def random_or_none(list_or_none: Optional) -> Optional:
    """
    Get a random element from a list if it is not empty.

    :param list_or_none: Either a list or None
    :return: random element if given a non-empty list, otherwise None
    """
    return random.choice(list_or_none) if list_or_none else list_or_none


def get_phrase_list(intent: str) -> List[str]:
    """
    Get all imported phrases for this intent as a list

    :param intent: name-parameter of the yml-section with which the phrases were imported
    :return: None if no phrases are known for this intent, otherwise list of phrases for this intent
    """
    return phrases.get(intent)


def get_random_phrase(intent: str) -> str:
    """
    Get a random phrase for this intent

    :param intent: name-parameter of the yml-section with which the phrases were imported
    :return: None if no phrases are known for this intent, otherwise a random element of the phrases for this intent
    """
    return random_or_none(phrases.get(intent))


def get_question_list(intent: str) -> List[str]:
    """
    Get all imported questions for this intent as a list

    :param intent: name-parameter of the yml-section with which the questions were imported
    :return: None if no questions are known for this intent, otherwise list of questions for this intent
    """
    return None if not qa.get(intent) else qa.get(intent).questions


def get_random_question(intent: str) -> str:
    """
    Get a random question for this intent

    :param intent: name-parameter of the yml-section with which the questions were imported
    :return: None if no questions are known for this intent, otherwise a random element of the questions for this intent
    """
    return random_or_none(get_question_list(intent))


def get_successful_answer_list(intent: str) -> List[str]:
    """
    Get all imported successful answers for this intent as a list

    :param intent: name-parameter of the yml-section with which the successful answers were imported
    :return: None if no successful answers are known for this intent, otherwise list of successful answers for this intent
    """
    return None if not qa.get(intent) else qa.get(intent).successful_answers


def get_random_successful_answer(intent: str) -> str:
    """
    Get a random successful answer for this intent

    :param intent: name-parameter of the yml-section with which the successful answers were imported
    :return: None if no successful answers are known for this intent,
        otherwise a random element of the successful answers for this intent
    """
    return random_or_none(get_successful_answer_list(intent))


def get_failure_answer_list(intent: str) -> List[str]:
    """
    Get all imported failure answers for this intent as a list

    :param intent: name-parameter of the yml-section with which the failure answers were imported
    :return: None if no failure answers are known for this intent, otherwise list of failure answers for this intent
    """
    return None if not qa.get(intent) else qa.get(intent).failure_answers


def get_random_failure_answer(intent: str) -> str:
    """
    Get a random failure answer for this intent

    :param intent: name-parameter of the yml-section with which the failure answers were imported
    :return: None if no failure answers are known for this intent,
        otherwise a random element of the failure answers for this intent
    """
    return random_or_none(get_failure_answer_list(intent))


def get_followup_question_list(intent: str) -> List[str]:
    """
    Get all imported followup questions for this intent as a list

    :param intent: name-parameter of the yml-section with which the followup questions were imported
    :return: None if no followup questions are known for this intent, otherwise list of followup questions for this intent
    """
    return None if not qa.get(intent) else qa.get(intent).followup_questions


def get_random_followup_question(intent: str) -> str:
    """
    Get a random followup question for this intent

    :param intent: name-parameter of the yml-section with which the followup questions were imported
    :return: None if no followup questions are known for this intent, 
        otherwise a random element of the followup questions for this intent
    """
    return random_or_none(get_followup_question_list(intent))


def get_followup_answer_list(intent: str) -> List[str]:
    """
    Get all imported followup answers for this intent as a list

    :param intent: name-parameter of the yml-section with which the followup answers were imported
    :return: None if no followup answers are known for this intent, otherwise list of followup answers for this intent
    """
    return None if not qa.get(intent) else qa.get(intent).followup_answers


def get_random_followup_answer(intent: str) -> str:
    """
    Get a random followup answer for this intent

    :param intent: name-parameter of the yml-section with which the followup answers were imported
    :return: None if no followup answers are known for this intent,
        otherwise a random element of the followup answers for this intent
    """
    return random_or_none(get_followup_answer_list(intent))
