from ravestate.testfixtures import *


CHILD_PROPERTY_NAME = 'child'
CHILD_PROPERTY_VALUE = 'I am a child'
GRANDCHILD_PROPERTY_NAME = 'grandchild'
GRANDCHILD_PROPERTY_VALUE = 'I am a grandchild'
CHILD_PROPERTY_ID = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}"
GRANDCHILD_PROPERTY_ID = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:{GRANDCHILD_PROPERTY_NAME}"


def test_add_state(mocker, context_fixture, context_with_property_fixture, state_fixture, context_wrapper_fixture):
    with mocker.patch.object(context_fixture, 'add_state'):
        context_wrapper_fixture.add_state(state_fixture)
        context_fixture.add_state.assert_called_once_with(st=state_fixture)


def test_context_shutdown(mocker, context_wrapper_fixture, context_fixture):
    with mocker.patch.object(context_fixture, 'shutdown'):
        context_wrapper_fixture.shutdown()
        context_fixture.shutdown.assert_called_once()


def test_context_shutting_down(mocker, context_wrapper_fixture, context_fixture):
    with mocker.patch.object(context_fixture, 'shutting_down'):
        context_wrapper_fixture.shutting_down()
        context_fixture.shutting_down.assert_called_once()


def test_property_push_pop(mocker, context_wrapper_fixture, context_with_property_fixture):
    with mocker.patch.object(context_with_property_fixture, 'emit'):
        # push child
        assert context_wrapper_fixture.push(parentpath=DEFAULT_PROPERTY_ID,
                                            child=PropertyBase(name=CHILD_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE))
        context_with_property_fixture.emit.assert_called_with(
            s(f"{DEFAULT_PROPERTY_ID}:pushed"),
            parents=None,
            wipe=False,
            payload=CHILD_PROPERTY_ID)

        # test child
        assert CHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))
        assert context_wrapper_fixture[CHILD_PROPERTY_ID] == DEFAULT_PROPERTY_VALUE
        context_wrapper_fixture[CHILD_PROPERTY_ID] = CHILD_PROPERTY_VALUE
        context_with_property_fixture.emit.assert_called_with(
            s(f"{CHILD_PROPERTY_ID}:changed"),
            parents=None,
            wipe=True,
            payload=CHILD_PROPERTY_VALUE)
        assert context_wrapper_fixture[CHILD_PROPERTY_ID] == CHILD_PROPERTY_VALUE

        # pop child
        assert context_wrapper_fixture.pop(CHILD_PROPERTY_ID)
        context_with_property_fixture.emit.assert_called_with(
            s(f"{DEFAULT_PROPERTY_ID}:popped"),
            parents=None,
            wipe=False,
            payload=CHILD_PROPERTY_ID)
        assert [] == list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))


def test_property_nested(mocker, context_wrapper_fixture, context_with_property_fixture):
    with mocker.patch.object(context_with_property_fixture, 'emit'):
        # push child
        assert context_wrapper_fixture.push(parentpath=DEFAULT_PROPERTY_ID,
                                            child=PropertyBase(name=CHILD_PROPERTY_NAME))
        context_with_property_fixture.emit.assert_called_with(
            s(f"{DEFAULT_PROPERTY_ID}:pushed"),
            parents=None,
            wipe=False,
            payload=CHILD_PROPERTY_ID)

        # push grandchild
        assert context_wrapper_fixture.push(parentpath=CHILD_PROPERTY_ID,
                                            child=PropertyBase(name=GRANDCHILD_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE))
        context_with_property_fixture.emit.assert_called_with(
            s(f"{CHILD_PROPERTY_ID}:pushed"),
            parents=None,
            wipe=False,
            payload=GRANDCHILD_PROPERTY_ID)

        # test children
        assert CHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))
        assert GRANDCHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(CHILD_PROPERTY_ID))
        assert context_wrapper_fixture[GRANDCHILD_PROPERTY_ID] == DEFAULT_PROPERTY_VALUE
        context_wrapper_fixture[GRANDCHILD_PROPERTY_ID] = GRANDCHILD_PROPERTY_VALUE
        context_with_property_fixture.emit.assert_called_with(
            s(f"{GRANDCHILD_PROPERTY_ID}:changed"),
            parents=None,
            wipe=True,
            payload=GRANDCHILD_PROPERTY_VALUE)
        assert context_wrapper_fixture[GRANDCHILD_PROPERTY_ID] == GRANDCHILD_PROPERTY_VALUE

        # pop child
        assert context_wrapper_fixture.pop(CHILD_PROPERTY_ID)
        context_with_property_fixture.emit.assert_called_with(
            s(f"{DEFAULT_PROPERTY_ID}:popped"),
            parents=None,
            wipe=False,
            payload=CHILD_PROPERTY_ID)
        assert [] == list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))


def test_property_nested_2(mocker, context_wrapper_fixture, context_with_property_fixture):
    with mocker.patch.object(context_with_property_fixture, 'emit'):
        # push child
        assert context_wrapper_fixture.push(parentpath=DEFAULT_PROPERTY_ID,
                                            child=PropertyBase(name=CHILD_PROPERTY_NAME))
        context_with_property_fixture.emit.assert_called_with(
            s(f"{DEFAULT_PROPERTY_ID}:pushed"),
            parents=None,
            wipe=False,
            payload=CHILD_PROPERTY_ID)

        # push grandchild
        assert context_wrapper_fixture.push(parentpath=CHILD_PROPERTY_ID,
                                            child=PropertyBase(name=GRANDCHILD_PROPERTY_NAME))
        context_with_property_fixture.emit.assert_called_with(
            s(f"{CHILD_PROPERTY_ID}:pushed"),
            parents=None,
            wipe=False,
            payload=GRANDCHILD_PROPERTY_ID)

        # test children
        assert CHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))
        assert GRANDCHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(CHILD_PROPERTY_ID))

        # pop grandchild
        assert context_wrapper_fixture.pop(GRANDCHILD_PROPERTY_ID)
        context_with_property_fixture.emit.assert_called_with(
            s(f"{CHILD_PROPERTY_ID}:popped"),
            parents=None,
            wipe=False,
            payload=GRANDCHILD_PROPERTY_ID)
        assert [] == list(context_wrapper_fixture.enum(CHILD_PROPERTY_ID))
        assert CHILD_PROPERTY_ID in list(context_wrapper_fixture.enum(DEFAULT_PROPERTY_ID))
