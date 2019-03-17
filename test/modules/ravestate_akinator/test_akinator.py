from pytest_mock import mocker


def test_react_to_play_asked(mocker):
    import ravestate_akinator
    test_dict = {}
    ravestate_akinator.play_ask(test_dict)

    assert test_dict["rawio:out"] == "Do you want to play 20 questions?"


def test_play_question_ignored(mocker):
    import ravestate_akinator
    test_dict = {}
    ravestate_akinator.akinator_play_question_ignored(test_dict)
    assert test_dict["rawio:out"] == "Oh well, maybe later!"


