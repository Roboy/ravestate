from ravestate.context import Context
from ravestate_conio import *
from ravestate.testfixtures import *


def test_console_input(context_wrapper_fixture: Context):
    context_wrapper_fixture.ctx._shutdown_flag.set()
    console_input(context_wrapper_fixture)


def test_console_output(capsys):
    expected = 'test'
    test_input = {"rawio:out": expected}
    console_output(test_input)
    captured = capsys.readouterr()
    assert captured.out == f"{expected}\n"
