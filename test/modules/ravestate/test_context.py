from ravestate.testfixtures import *
from reggol import strip_prefix
from testfixtures import LogCapture


def test_emit(context_fixture):
    context_fixture.emit(s(DEFAULT_PROPERTY_CHANGED))
    assert len(context_fixture._spikes) == 1


def test_run(mocker, context_fixture):
    context_fixture.emit = mocker.stub()
    context_fixture.shutdown_flag = True
    context_fixture.run()
    context_fixture.emit.assert_called_once_with(s(':startup'))
    context_fixture.shutdown()


def test_run_error(context_fixture):
    context_fixture.shutdown_flag = True
    context_fixture.run()
    with LogCapture(attributes=strip_prefix) as log_capture:
        context_fixture.run()
        log_capture.check('Attempt to start context twice!')
    context_fixture.shutdown()


def test_shutting_down(context_fixture):
    assert context_fixture.shutting_down() is False
    context_fixture._shutdown_flag.set()
    assert context_fixture.shutting_down() is True


def test_shutdown(mocker, context_fixture):
    context_fixture._run_task = mocker.Mock()
    mocker.patch.object(context_fixture._run_task, 'join')
    context_fixture.emit = mocker.stub(name='emit')
    context_fixture.shutdown()
    context_fixture.emit.assert_called_once_with(s(':shutdown'))
    context_fixture._run_task.join.assert_called_once()


def test_add_module_new(mocker, context_fixture):
    from ravestate import registry
    with mocker.patch('ravestate.registry.has_module', return_value=False):
        with mocker.patch('ravestate.registry.import_module'):
            context_fixture.add_module(DEFAULT_MODULE_NAME)
            registry.import_module.assert_called_once_with(
                module_name=DEFAULT_MODULE_NAME,
                callback=context_fixture._module_registration_callback
            )


def test_add_module_present(mocker, context_fixture):
    with mocker.patch('ravestate.registry.has_module', return_value=True):
        with mocker.patch.object(context_fixture, '_module_registration_callback'):
            context_fixture.add_module(DEFAULT_MODULE_NAME)
            context_fixture._module_registration_callback.assert_called_once()


def test_remove_dependent_state(context_fixture: Context, state_fixture: State):
    prop = PropertyBase(name=DEFAULT_PROPERTY_NAME)
    prop.set_parent_path(DEFAULT_MODULE_NAME)
    context_fixture.add_prop(prop=prop)
    context_fixture.add_state(st=state_fixture)
    assert state_fixture in context_fixture._states
    assert prop.fullname() in context_fixture._properties
    context_fixture.rm_prop(prop=prop)
    assert state_fixture not in context_fixture._states
    assert prop.fullname() not in context_fixture._properties


def test_add_state():
    pass
