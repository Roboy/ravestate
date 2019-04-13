import pytest

from ravestate_nlp import yes_no, spacy_nlp_en


def create_doc(text: str):
    if not text:
        return None
    return spacy_nlp_en(text)


@pytest.mark.parametrize("test_input, expected",
                         [("y", "yes"), ("yeah", "yes"),
                          ("nope", "no"), ("certainly not", "no"),
                          ("probably", "p"), ("likely", "p"),
                          ("probably not", "pn"), ("not likely", "pn"),
                          ("i do not know", "idk"), ("i do not have a clue", "idk"),
                          ("Hey ho!", "_")])
def test_yes(test_input, expected):
    assert yes_no(create_doc(test_input)) == expected
