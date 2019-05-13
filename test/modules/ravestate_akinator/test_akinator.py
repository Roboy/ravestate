from pytest_mock import mocker


def test_react_to_play_asked(mocker):
    import ravestate_akinator as akin
    import ravestate_rawio as rawio
    test_dict = {}
    akin.play_ask(test_dict)
    assert test_dict[rawio.prop_out] == "Do you want to play 20 questions?"


def test_play_question_ignored(mocker):
    import ravestate_akinator as akin
    import ravestate_rawio as rawio
    test_dict = {}
    akin.akinator_play_question_ignored(test_dict)
    assert test_dict[rawio.prop_out] == "Oh well, maybe later!"


