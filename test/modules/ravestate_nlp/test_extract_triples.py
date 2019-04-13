import pytest
import spacy
from ravestate_nlp import *

@pytest.fixture
def spacy_model():
    nlp = spacy_nlp_en
    from spacy.tokens import Doc
    if not Doc.has_extension('triples'):
        Doc.set_extension('triples', getter=extract_triples)
    return nlp


def basic_test(expected_triples, spacy_model, text_input):
    triples = spacy_model(text_input)._.triples
    print(triples)
    assert expected_triples == triples


@pytest.mark.parametrize('text_input, expected_triples',
                         [('I like pizza', [('I', 'like', 'pizza')]),
                          ('You have cake', [('you', 'have', 'cake')])]
                         )
def test_basic_triples(spacy_model, text_input, expected_triples):
    basic_test(expected_triples, spacy_model, text_input)


@pytest.mark.parametrize('text_input, expected_triples',
                         [('I like pizza and you have cake', [('I', 'like', 'pizza'), ('you', 'have', 'cake')])]
                         )
def test_multi_triples(spacy_model, text_input, expected_triples):
    basic_test(expected_triples, spacy_model, text_input)


@pytest.mark.parametrize('text_input, expected_triples',
                         [('I am a student', [('I', 'be', 'student')]),
                          ('My name is Roboy', [('name', 'be', 'Roboy')])]
                         )
def test_attr_triples(spacy_model, text_input, expected_triples):
    basic_test(expected_triples, spacy_model, text_input)


@pytest.mark.parametrize('text_input, expected_triples',
                         [('Who are you', [('you', 'be', QuestionWord.PERSON)]),
                          ('Where am I', [('I', 'be', QuestionWord.PLACE)])]
                         )
def test_questions(spacy_model, text_input, expected_triples):
    basic_test(expected_triples, spacy_model, text_input)
