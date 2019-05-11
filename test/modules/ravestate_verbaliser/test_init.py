from os.path import join, dirname, realpath

from pytest_mock import mocker


def test_react_to_intent(mocker):
    from ravestate_verbaliser import verbaliser
    verbaliser.add_file(join(dirname(realpath(__file__)),
                                                  "verbaliser_testfiles", "phrases_test.yml"))
    import ravestate_verbaliser
    import ravestate_rawio as rawio
    test_dict = {'verbaliser:intent:changed': 'test1'}
    ravestate_verbaliser.react_to_intent(test_dict)
    assert test_dict[rawio.prop_out] in ravestate_verbaliser.verbaliser.get_phrase_list('test1')


def test_react_to_intent_no_phrase(mocker):
    from ravestate_verbaliser import verbaliser
    verbaliser.add_file(join(dirname(realpath(__file__)),
                                                  "verbaliser_testfiles", "phrases_test.yml"))
    import ravestate_verbaliser
    test_dict = {'verbaliser:intent:changed': 'test'}
    ravestate_verbaliser.react_to_intent(test_dict)
    assert "rawio:out" not in test_dict
