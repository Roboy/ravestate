import pytest
import spacy
from ravestate_nlp import extract_triples
from ravestate_nlp.question_words import QuestionWord


@pytest.fixture
def spacy_model():
    nlp = spacy.load('en_core_web_sm')
    from spacy.tokens import Doc
    if not Doc.has_extension('triple'):
        Doc.set_extension('triple', getter=extract_triples)
    return nlp


def basic_test(expected_triples, spacy_model, text_input):
    triples = spacy_model(text_input)._.triple
    print(triples)
    assert triples == expected_triples


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
