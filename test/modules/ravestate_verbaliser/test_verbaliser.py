from os.path import join, dirname, realpath
from typing import List

from ravestate.property import PropertyBase
from ravestate_verbaliser import verbaliser


def cleanup_verbaliser():
    """
    Clean the dicts of the verbaliser to test with empty dicts
    """
    verbaliser.qa = {}
    verbaliser.phrases = {}


def assert_list_equ(actual: List, expected: List):
    """
    Asserts that two lists contain the same elements
    """
    assert all([a == b for a, b in (zip(actual, expected))])


def assert_list_and_random(actual: List, expected: List, random_element: str):
    """
    Asserts that the two lists contain the same elements and
    also asserts that the random_element is in the expected list
    """
    assert_list_equ(actual, expected)
    assert random_element in expected


def test_add_file_phrases():
    verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "phrases_test.yml"))
    assert_phrases_file()
    cleanup_verbaliser()


def assert_phrases_file():
    assert_list_and_random(verbaliser.get_phrase_list('test1'), ['a', 'b', 'c', 'd'],
                           verbaliser.get_random_phrase('test1'))
    assert_list_and_random(verbaliser.get_phrase_list('test2'), ['a'],
                           verbaliser.get_random_phrase('test2'))
    assert_list_and_random(verbaliser.get_phrase_list('test3'), ['a', 'b'],
                           verbaliser.get_random_phrase('test3'))


def test_add_file_qa():
    verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "QAList_test.yml"))
    assert_qa_file()
    cleanup_verbaliser()


def assert_qa_file():
    # section INTENT
    assert_list_and_random(verbaliser.get_question_list('INTENT'), ['Q1', 'Q2', 'Q3'],
                           verbaliser.get_random_question('INTENT'))
    assert_list_and_random(verbaliser.get_successful_answer_list('INTENT'), ['SA1', 'SA2'],
                           verbaliser.get_random_successful_answer('INTENT'))
    assert_list_and_random(verbaliser.get_failure_answer_list('INTENT'), ['FA1'],
                           verbaliser.get_random_failure_answer('INTENT'))
    assert_list_and_random(verbaliser.get_followup_question_list('INTENT'), ['FUPQ1'],
                           verbaliser.get_random_followup_question('INTENT'))
    assert_list_and_random(verbaliser.get_followup_answer_list('INTENT'), ['FUPA1'],
                           verbaliser.get_random_followup_answer('INTENT'))

    # section INTENT2
    assert_list_and_random(verbaliser.get_question_list('INTENT2'), ['Q1'],
                           verbaliser.get_random_question('INTENT2'))
    assert not verbaliser.get_successful_answer_list('INTENT2')
    assert not verbaliser.get_random_successful_answer('INTENT2')
    assert_list_and_random(verbaliser.get_failure_answer_list('INTENT2'), ['FA1', 'FA2'],
                           verbaliser.get_random_failure_answer('INTENT2'))
    assert_list_and_random(verbaliser.get_followup_question_list('INTENT2'), ['FUPQ1', 'FUPQ2'],
                           verbaliser.get_random_followup_question('INTENT2'))
    assert_list_and_random(verbaliser.get_followup_answer_list('INTENT2'), ['FUPA1'],
                           verbaliser.get_random_followup_answer('INTENT2'))


def test_add_file_mixed():
    verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "mixed_test.yml"))
    assert_mixed_file()
    cleanup_verbaliser()


def assert_mixed_file():
    assert_list_and_random(verbaliser.get_phrase_list('mixed_phrases1'), ['a', 'b', 'c', 'd'],
                           verbaliser.get_random_phrase('mixed_phrases1'))
    assert_list_and_random(verbaliser.get_phrase_list('mixed_phrases2'), ['a'],
                           verbaliser.get_random_phrase('mixed_phrases2'))

    assert_list_and_random(verbaliser.get_question_list('mixed_INTENT'), ['Q1', 'Q2', 'Q3'],
                           verbaliser.get_random_question('mixed_INTENT'))
    assert_list_and_random(verbaliser.get_successful_answer_list('mixed_INTENT'), ['SA1', 'SA2'],
                           verbaliser.get_random_successful_answer('mixed_INTENT'))
    assert_list_and_random(verbaliser.get_failure_answer_list('mixed_INTENT'), ['FA1'],
                           verbaliser.get_random_failure_answer('mixed_INTENT'))
    assert_list_and_random(verbaliser.get_followup_question_list('mixed_INTENT'), ['FUPQ1'],
                           verbaliser.get_random_followup_question('mixed_INTENT'))
    assert_list_and_random(verbaliser.get_followup_answer_list('mixed_INTENT'), ['FUPA1'],
                           verbaliser.get_random_followup_answer('mixed_INTENT'))


def test_add_file_nosection():
    verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "nosection_test.yml"))
    assert_nosection_file()
    cleanup_verbaliser()


def assert_nosection_file():
    assert_list_and_random(verbaliser.get_phrase_list('nosection1'), ['a', 'b', 'c', 'd'],
                           verbaliser.get_random_phrase('nosection1'))


def test_add_file_wrongformat():
    assert not verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "wrongformat_test.yml"))


def test_add_folder():
    verbaliser.add_folder(join(dirname(realpath(__file__)), "verbaliser_testfiles"))
    assert_phrases_file()
    assert_qa_file()
    assert_mixed_file()
    assert_nosection_file()
    cleanup_verbaliser()


def test_add_file_nonexistent():
    assert not verbaliser.add_file(join(dirname(realpath(__file__)), "verbaliser_testfiles", "nonexistent.yml"))
