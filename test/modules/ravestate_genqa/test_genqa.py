from ravestate.context import Context
from ravestate.state import Delete
from ravestate.testfixtures import *
from pytest_mock import mocker
from testfixtures import LogCapture


FILE_NAME = 'ravestate_genqa'
PREFIX = f"[{FILE_NAME}] [\x1b[1;36m{FILE_NAME}\x1b[0m]"


def test_hello_world_genqa(mocker, context_wrapper_fixture: Context):
    with LogCapture() as capture:
        import ravestate_genqa
        result = ravestate_genqa.hello_world_genqa(context_wrapper_fixture)
        expected = 'Server address is not set. Shutting down GenQA.'
        capture.check_present((f"{FILE_NAME}", '\x1b[1;31mERROR\x1b[0m', f"{PREFIX} {expected}"))
        assert isinstance(result, Delete)
        # TODO: Write test for server address present


@pytest.mark.skip(reason="test broken")
def test_drqa_module(mocker, context_wrapper_fixture: Context):
    with LogCapture() as capture:
        verbalizer_mock = mocker.patch('ravestate_verbaliser.verbaliser.get_random_phrase')
        import ravestate_genqa
        result = ravestate_genqa.drqa_module(context_wrapper_fixture)
        verbalizer_mock.assert_called_once_with("question-answering-starting-phrases")
        expected = 'Server address is not set. Shutting down GenQA.'
        capture.check_present((f"{FILE_NAME}", '\x1b[1;31mERROR\x1b[0m', f"{PREFIX} {expected}"))
        assert isinstance(result, Delete)
        # TODO: Write test for server address present

