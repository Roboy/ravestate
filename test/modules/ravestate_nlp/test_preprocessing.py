import pytest
from ravestate_nlp import nlp_preprocess
from testfixtures import log_capture

FILE_NAME = 'ravestate_nlp'


@pytest.fixture
def basic_input():
    return {'rawio:in': 'Hello world my name is Roboy'}


@log_capture()
def test_tokenization(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = ('hello', 'world', 'my', 'name', 'is', 'roboy')
    assert basic_input["nlp:tokens"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:tokens]: {expected}"))


@log_capture()
def test_postags(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = ('INTJ', 'NOUN', 'DET', 'NOUN', 'VERB', 'ADJ')
    assert basic_input["nlp:postags"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:postags]: {expected}"))


@log_capture()
def test_lemmas(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = ('hello', 'world', '-PRON-', 'name', 'be', 'roboy')
    assert basic_input["nlp:lemmas"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:lemmas]: {expected}"))


@log_capture()
def test_tags(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = ('UH', 'NN', 'PRP$', 'NN', 'VBZ', 'JJ')
    assert basic_input["nlp:tags"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:tags]: {expected}"))


@pytest.mark.skip(reason="Spacy NER is too unreliable for testing")
@log_capture()
def test_ner(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = (('roboy', 'ORG'),)
    assert basic_input["nlp:ner"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:ner]: {expected}"))


@log_capture()
def test_roboy(capture, basic_input):
    nlp_preprocess(basic_input)
    expected = True
    assert basic_input["nlp:roboy"] == expected
    capture.check_present((f"{FILE_NAME}", 'INFO', f"[NLP:roboy]: {expected}"))
