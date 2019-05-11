from ravestate.testfixtures import *


def test_emit(context_fixture, spike_fixture):
    sig = SignalRef(DEFAULT_PROPERTY_CHANGED)
    context_fixture.emit(sig)
    assert len(context_fixture._spikes_per_signal[sig]) == 1
    list(context_fixture._spikes_per_signal[sig])[0].adopt(spike_fixture)
    context_fixture.emit(sig, wipe=True)
    assert len(context_fixture._spikes_per_signal[sig]) == 2


def test_run(mocker, context_fixture):
    context_fixture.emit = mocker.stub()
    context_fixture.shutdown_flag = True
    context_fixture.run()
    context_fixture.emit.assert_called_once_with(sig_startup)
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
    context_fixture.emit.assert_called_once_with(sig_shutdown)
    context_fixture._run_task.join.assert_called_once()


def test_add_module_new(mocker, context_fixture):
    with mocker.patch('ravestate.context.has_module', return_value=False):
        with mocker.patch('ravestate.context.import_module'):
            from ravestate.context import import_module
            context_fixture.add_module(DEFAULT_MODULE_NAME)
            import_module.assert_called_once_with(
                module_name=DEFAULT_MODULE_NAME,
                callback=context_fixture._module_registration_callback
            )


def test_add_module_present(mocker, context_fixture):
    with mocker.patch('ravestate.context.has_module', return_value=True):
        with mocker.patch.object(context_fixture, '_module_registration_callback'):
            context_fixture.add_module(DEFAULT_MODULE_NAME)
            context_fixture._module_registration_callback.assert_called_once()


def test_remove_dependent_state(context_fixture: Context, state_fixture: State):
    prop = Property(name=DEFAULT_PROPERTY_NAME)
    prop.set_parent_path(DEFAULT_MODULE_NAME)
    context_fixture.add_prop(prop=prop)
    context_fixture.add_state(st=state_fixture)
    assert state_fixture in context_fixture._activations_per_state
    assert prop.id() in context_fixture._properties
    context_fixture.rm_prop(prop=prop)
    assert state_fixture not in context_fixture._activations_per_state
    assert prop.id() not in context_fixture._properties


def test_remove_unknown_state(context_fixture: Context, state_fixture: State):
    with LogCapture(attributes=strip_prefix) as log_capture:
        context_fixture.rm_state(st=state_fixture)
        log_capture.check(f"Attempt to remove unknown state `{state_fixture.name}`!")


def test_remove_state_with_signal(context_with_property_fixture: Context, state_signal_a_fixture: State):
    context_with_property_fixture.add_state(st=state_signal_a_fixture)
    assert state_signal_a_fixture.signal in context_with_property_fixture._needy_acts_per_state_per_signal
    context_with_property_fixture.rm_state(st=state_signal_a_fixture)
    assert state_signal_a_fixture.signal not in context_with_property_fixture._needy_acts_per_state_per_signal


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
    assert SignalRef(DEFAULT_PROPERTY_CHANGED) in \
           context_with_property_fixture._signal_causes[state_signal_a_fixture.signal][0]

    context_with_property_fixture.add_state(st=state_signal_b_fixture)
    context_with_property_fixture.add_state(st=state_signal_c_fixture)
    context_with_property_fixture.add_state(st=state_signal_d_fixture)
    assert len(context_with_property_fixture._activations_per_state) == 5

    # Make sure, that d's constraint was completed correctly
    d_conjunctions = list(state_signal_d_fixture.completed_constraint.conjunctions())
    assert len(d_conjunctions) == 4  # 2 completed, 2 uncompleted
    d_conjunctions = [
        conj for conj in d_conjunctions
        if len(tuple(conj.signals())) > 2]
    assert len(d_conjunctions) == 2  # 2 completed
    assert SignalRef(DEFAULT_PROPERTY_CHANGED) in d_conjunctions[0]
    assert state_signal_a_fixture.signal in d_conjunctions[0]
    assert SignalRef(DEFAULT_PROPERTY_CHANGED) in d_conjunctions[1]
    assert state_signal_a_fixture.signal in d_conjunctions[1]
    assert \
        state_signal_b_fixture.signal in d_conjunctions[0] and state_signal_c_fixture.signal in d_conjunctions[1] or \
        state_signal_b_fixture.signal in d_conjunctions[1] and state_signal_c_fixture.signal in d_conjunctions[0]

    # Basic specificity sanity checks
    assert len(list(context_with_property_fixture._states_for_signal(SignalRef(DEFAULT_PROPERTY_CHANGED)))) == 5
    a_acts = list(context_with_property_fixture._state_activations(st=state_signal_a_fixture))
    b_acts = list(context_with_property_fixture._state_activations(st=state_signal_b_fixture))
    c_acts = list(context_with_property_fixture._state_activations(st=state_signal_c_fixture))
    d_acts = list(context_with_property_fixture._state_activations(st=state_signal_d_fixture))
    assert len(a_acts) == 1
    assert len(b_acts) == 1
    assert len(c_acts) == 1
    assert len(d_acts) == 1
    propchange_sig_spec = context_with_property_fixture.signal_specificity(SignalRef(DEFAULT_PROPERTY_CHANGED))
    assert a_acts[0].specificity() == propchange_sig_spec
    a_sig_spec = context_with_property_fixture.signal_specificity(state_signal_a_fixture.signal)
    assert a_sig_spec == 1/3
    assert b_acts[0].specificity() == a_sig_spec
    assert d_acts[0].specificity() == 1.0


def test_add_state_configurable_age(context_with_property_fixture: Context):
    my_cond = SignalRef(DEFAULT_PROPERTY_CHANGED, min_age=ConfigurableAge(key="min_age_key"),
                     max_age=ConfigurableAge(key="max_age_key"))

    @state(cond=my_cond)
    def conf_st(ctx):
        pass
    conf_st.module_name = DEFAULT_MODULE_NAME
    context_with_property_fixture._config.add_conf(mod=Module(name=DEFAULT_MODULE_NAME,
                                                              config={"min_age_key": 2.5, "max_age_key": 4.5}))
    context_with_property_fixture.add_state(st=conf_st)
    assert my_cond.min_age_value == 2.5
    assert my_cond.max_age_value == 4.5


def test_add_state_configurable_age_not_in_config(context_with_property_fixture: Context):
    my_cond = SignalRef(
        DEFAULT_PROPERTY_CHANGED,
        min_age=ConfigurableAge(key="min_age_key"),
        max_age=ConfigurableAge(key="max_age_key"))

    @state(cond=my_cond)
    def conf_st(ctx):
        pass

    conf_st.module_name = DEFAULT_MODULE_NAME
    context_with_property_fixture.add_state(st=conf_st)
    assert my_cond.min_age_value == 0.
    assert my_cond.max_age_value == 5.


def test_add_state_unknown_property(context_fixture: Context, state_fixture: State):
    with LogCapture(attributes=strip_prefix) as log_capture:
        context_fixture.add_state(st=state_fixture)
        log_capture.check(f"Attempt to add state which depends on unknown property `{DEFAULT_PROPERTY_ID}`!")


def test_add_prop_twice(context_fixture: Context):
    with LogCapture(attributes=strip_prefix) as log_capture:
        prop = Property(name=DEFAULT_PROPERTY_NAME)
        prop.set_parent_path(DEFAULT_MODULE_NAME)
        context_fixture.add_prop(prop=prop)
        context_fixture.add_prop(prop=prop)
        log_capture.check(f"Attempt to add property {DEFAULT_PROPERTY_ID} twice!")


def test_get_unknown_property(context_fixture: Context):
    with LogCapture(attributes=strip_prefix) as log_capture:
        assert context_fixture[DEFAULT_PROPERTY_ID] is None
        log_capture.check(f"Attempt to retrieve unknown property by key `{DEFAULT_PROPERTY_ID}`!")


def test_create_and_run(mocker):
    with mocker.patch.object(Context, 'run'):
        create_and_run_context()
        Context.run.assert_called_once()


def test_overriding_config(mocker):
    with mocker.patch.object(Configuration, 'set'):
        ctx = Context("-d", DEFAULT_MODULE_NAME, "key", "new_val")
        Configuration.set.assert_called_once_with(DEFAULT_MODULE_NAME, "key", "new_val")


def test_runtime_overriding_config(mocker):
    with mocker.patch.object(Configuration, 'set'):
        ctx = Context(runtime_overrides=[(DEFAULT_MODULE_NAME, "key", "new_runtime_val")])
        Configuration.set.assert_called_once_with(DEFAULT_MODULE_NAME, "key", "new_runtime_val")


def test_invalid_tickrate():
    with LogCapture(attributes=strip_prefix) as log_capture:
        ctx = Context(runtime_overrides=[(CORE_MODULE_NAME, TICK_RATE_CONFIG_KEY, 0)])
        log_capture.check_present("Attempt to set core config `tickrate` to a value less-than 1!")
        assert ctx.tick_rate == 1
