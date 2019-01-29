from pytest_mock import mocker


def test_react_to_pushed_interloc(mocker):
    import ravestate_hibye
    test_dict = {}
    ravestate_hibye.react_to_pushed_interloc(test_dict)
    assert test_dict["verbaliser:intent"] == "greeting"


def test_react_to_popped_interloc(mocker):
    test_dict = {}
    import ravestate_hibye
    ravestate_hibye.react_to_popped_interloc(test_dict)
    assert test_dict["verbaliser:intent"] == "farewells"
