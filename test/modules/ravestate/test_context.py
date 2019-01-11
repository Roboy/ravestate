from ravestate.testfixtures import *


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


def test_add_state(
        context_with_property_fixture: Context,
        state_fixture: State,
        state_signal_a_fixture: State,
        state_signal_b_fixture: State,
        state_signal_c_fixture: State,
        state_signal_d_fixture: State):

    context_with_property_fixture.add_state(st=state_signal_a_fixture)
    with LogCapture(attributes=strip_prefix) as log_capture:
        context_with_property_fixture.add_state(st=state_signal_a_fixture)
        log_capture.check(f"Attempt to add state `{state_signal_a_fixture.name}` twice!")

    # Add a cyclic cause for module:property:changed, make sure shit doesn't explode
    context_with_property_fixture.add_state(st=state_fixture)

    # Make sure, that module:property:changed was added as a cause for module:a
    assert s(DEFAULT_PROPERTY_CHANGED) in \
        context_with_property_fixture._signal_causes[state_signal_a_fixture.signal()][0]

    context_with_property_fixture.add_state(st=state_signal_b_fixture)
    context_with_property_fixture.add_state(st=state_signal_c_fixture)
    context_with_property_fixture.add_state(st=state_signal_d_fixture)
    assert len(context_with_property_fixture._activations_per_state) == 5

    # Make sure, that d's constraint was completed correctly
    d_conjunctions = list(state_signal_d_fixture.constraint.conjunctions())
    assert len(d_conjunctions) == 2
    assert s(DEFAULT_PROPERTY_CHANGED) in d_conjunctions[0]
    assert state_signal_a_fixture.signal() in d_conjunctions[0]
    assert s(DEFAULT_PROPERTY_CHANGED) in d_conjunctions[1]
    assert state_signal_a_fixture.signal() in d_conjunctions[1]
    assert \
        state_signal_b_fixture.signal() in d_conjunctions[0] and state_signal_c_fixture.signal() in d_conjunctions[1] or \
        state_signal_b_fixture.signal() in d_conjunctions[1] and state_signal_c_fixture.signal() in d_conjunctions[0]

    # Basic specificity sanity checks
    assert len(list(context_with_property_fixture._states_for_signal(s(DEFAULT_PROPERTY_CHANGED)))) == 5
    a_acts = list(context_with_property_fixture._state_activations(st=state_signal_a_fixture))
    b_acts = list(context_with_property_fixture._state_activations(st=state_signal_b_fixture))
    c_acts = list(context_with_property_fixture._state_activations(st=state_signal_c_fixture))
    d_acts = list(context_with_property_fixture._state_activations(st=state_signal_d_fixture))
    assert len(a_acts) == 1
    assert len(b_acts) == 1
    assert len(c_acts) == 1
    assert len(d_acts) == 1
    propchange_sig_spec = context_with_property_fixture.signal_specificity(s(DEFAULT_PROPERTY_CHANGED))
    assert a_acts[0].specificity() == propchange_sig_spec
    a_sig_spec = context_with_property_fixture.signal_specificity(state_signal_a_fixture.signal())
    assert a_sig_spec == 1/3
    assert b_acts[0].specificity() == a_sig_spec + propchange_sig_spec
    assert 1.53 < d_acts[0].specificity() < 1.54

