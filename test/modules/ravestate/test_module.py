from ravestate.testfixtures import *
from ravestate.module import Module, import_module


def test_illegal_nested_modules():
    with LogCapture(attributes=strip_prefix) as log_capture:
        with Module(name="a"):
            with Module(name="b"):
                pass
        log_capture.check("Nested `with Module(...)` calls are not supported!`")


def test_add_list_of_props(state_signal_a_fixture, state_signal_b_fixture):
    with Module(name=DEFAULT_MODULE_NAME) as mod:
        mod.add([state_signal_a_fixture, state_signal_b_fixture])
        assert len(mod.states) == 2
        assert state_signal_a_fixture in mod.states
        assert state_signal_b_fixture in mod.states


def test_add_invalid_argument():
    with LogCapture(attributes=strip_prefix) as log_capture:
        with Module(name="Invalid Argument") as mod:
            mod.add(42)
        log_capture.check("Module.add() called with invalid argument 42!")


def test_import_module(mocker):
    with mocker.patch('importlib.import_module'):
        from importlib import import_module as lib_import
        import_module(module_name=DEFAULT_MODULE_NAME, callback=None)
        lib_import.assert_called_once_with(DEFAULT_MODULE_NAME)
