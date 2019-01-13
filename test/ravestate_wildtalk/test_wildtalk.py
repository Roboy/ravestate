from pytest_mock import mocker


def test_wildtalk_state(mocker):
    import_mock = mocker.Mock()
    mocker.patch.dict('sys.modules', {'roboy_parlai': import_mock})
    wildtalk_mock = mocker.patch('roboy_parlai.wildtalk', return_value='test')
    registry_mock = mocker.patch('ravestate.registry.register')
    import ravestate_wildtalk
    registry_mock.assert_called_with(name="wildtalk",
                                     states=(ravestate_wildtalk.wildtalk_state,))
    test_dict = {"rawio:in": 'test'}
    ravestate_wildtalk.wildtalk_state(test_dict)
    assert test_dict["rawio:out"] == 'test'
