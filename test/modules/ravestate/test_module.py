from ravestate.testfixtures import *
from ravestate.module import Module


def test_illegal_nested_modules():
    with LogCapture(attributes=strip_prefix) as log_capture:
        with Module(name="a"):
            with pytest.raises(RuntimeError) as exc:
                with Module(name="b"):
                    pass
            assert str(exc.value) == "Nested `with Module(...)` calls are not supported!`"


def test_add_list_of_props(state_signal_a_fixture, state_signal_b_fixture):
    with Module(name=DEFAULT_MODULE_NAME) as mod:
        mod.add([state_signal_a_fixture, state_signal_b_fixture])
        assert len(mod.states) == 2
        assert state_signal_a_fixture in mod.states
        assert state_signal_b_fixture in mod.states
