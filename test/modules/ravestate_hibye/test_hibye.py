from pytest_mock import mocker
from ravestate_verbaliser import intent


def test_react_to_pushed_interloc(mocker):
    import ravestate_hibye
    test_dict = {}
    ravestate_hibye.greeting(test_dict)
    assert test_dict[intent] == "greeting"


def test_react_to_popped_interloc(mocker):
    test_dict = {}
    import ravestate_hibye
    ravestate_hibye.farewell(test_dict)
    assert test_dict[intent] == "farewells"
