import pytest

from ravestate_nlp import yes_no, spacy_nlp_en


def create_doc(text: str):
    if not text:
        return None
    return spacy_nlp_en(text)


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", 2),
                          ("nope", -2),
                          ("probably", 1),
                          ("not certainly", -2),
                          ("probably not", -1),
                          ("i do not know", 0),
                          ("Hey ho!", None)])
def test_yes(test_input, expected_value):
    assert expected_value == yes_no(create_doc(test_input)).yes()


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah, thank you", 2),
                          ("thanks but nope", -2),
                          ("well probably", 1),
                          ("not certainly but ok", -2),
                          ("probably not today, sorry", -1),
                          ("well to be honest i do not know", 0)])
def test_yes_sentence(test_input, expected_value):
    assert expected_value == yes_no(create_doc(test_input)).yes()


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", -2),
                          ("nope", 2),
                          ("probably", -1),
                          ("not certainly", 2),
                          ("probably not", 1),
                          ("i do not know", 0),
                          ("Hey ho!", None)])
def test_no(test_input, expected_value):
    assert expected_value == yes_no(create_doc(test_input)).no()


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah, no problem", -2),
                          ("nope, never in a million years", 2),
                          ("i just never know", 0),
                          ("probably, thanks", -1),
                          ("not certainly but ok", 2),
                          ("probably not today man", 1),
                          ("i do not know that right now", 0),
                          ("Hey ho!", None)])
def test_no_sentence(test_input, expected_value):
    assert expected_value == yes_no(create_doc(test_input)).no()


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", False),
                          ("nope", False),
                          ("probably", False),
                          ("not certainly", False),
                          ("probably not", False),
                          ("i do not know", True),
                          ("Hey ho!", None)])
def test_unk(test_input, expected_value):
    assert expected_value == yes_no(create_doc(test_input)).unk()


