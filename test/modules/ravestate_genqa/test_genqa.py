from ravestate.context import Context
from ravestate.state import Delete
from ravestate.testfixtures import *
from pytest_mock import mocker
from testfixtures import LogCapture


FILE_NAME = 'ravestate_genqa'


def test_hello_world_genqa(mocker, context_wrapper_fixture: Context):
    with LogCapture(attributes=strip_prefix) as capture:
        import ravestate_genqa
        result = ravestate_genqa.hello_world_genqa(context_wrapper_fixture)
        capture.check_present('Server address is not set. Shutting down GenQA.')
        assert isinstance(result, Delete)
        # TODO: Write test for server address present


@pytest.mark.skip(reason="test broken")
def test_drqa_module(mocker, context_wrapper_fixture: Context):
    with LogCapture(attributes=strip_prefix) as capture:
        verbalizer_mock = mocker.patch('ravestate_verbaliser.verbaliser.get_random_phrase')
        import ravestate_genqa
        result = ravestate_genqa.drqa_module(context_wrapper_fixture)
        verbalizer_mock.assert_called_once_with("question-answering-starting-phrases")
        capture.check_present('Server address is not set. Shutting down GenQA.')
        assert isinstance(result, Delete)
        # TODO: Write test for server address present

