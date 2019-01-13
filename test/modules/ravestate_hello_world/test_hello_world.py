from pytest_mock import mocker


def test_hello_world(mocker):
    registry_mock = mocker.patch('ravestate.registry.register')
    import ravestate_hello_world
    registry_mock.assert_called_with(name="hi",
                                     states=(ravestate_hello_world.hello_world,
                                             ravestate_hello_world.generic_answer,
                                             ravestate_hello_world.face_recognized))
    test_dict = {}
    ravestate_hello_world.hello_world(test_dict)
    assert test_dict["verbaliser:intent"] == "greeting"


def test_generic_answer():
    import ravestate_hello_world
    test_dict = {"rawio:in": 'test'}
    ravestate_hello_world.generic_answer(test_dict)
    assert test_dict["rawio:out"] == f"Your input contains {len('test')} characters!"


def test_face_recognized():
    import ravestate_hello_world
    test_dict = {"facerec:face": 'test'}
    ravestate_hello_world.face_recognized(test_dict)
    assert test_dict["rawio:out"] == f"I see you, {'test'}!"
