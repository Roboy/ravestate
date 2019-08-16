import pytest

from ravestate_nlp import yes_no, spacy_nlp_en


def create_doc(text: str):
    if not text:
        return None
    return spacy_nlp_en(text)


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", 2),
                          ("nope", False),
                          ("probably", 1),
                          ("not certainly", False),
                          ("probably not", False),
                          ("i do not know", False),
                          ("Hey ho!", None)])
def test_yes(test_input, expected_value):
    assert yes_no(create_doc(test_input)).yes() == expected_value


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", False),
                          ("nope", 2),
                          ("probably", False),
                          ("not certainly", 2),
                          ("probably not", 1),
                          ("i do not know", False),
                          ("Hey ho!", None)])
def test_no(test_input, expected_value):
    assert yes_no(create_doc(test_input)).no() == expected_value


@pytest.mark.parametrize("test_input, expected_value",
                         [("yeah", False),
                          ("nope", False),
                          ("probably", False),
                          ("not certainly", False),
                          ("probably not", False),
                          ("i do not know", True),
                          ("Hey ho!", None)])
def test_unk(test_input, expected_value):
    assert yes_no(create_doc(test_input)).unk() == expected_value


