from pytest_mock import mocker
from ravestate_rawio import prop_in as raw_in, prop_out as raw_out

# TODO adapt test to new server/client architecture
# def test_wildtalk_state(mocker):
#     import_mock = mocker.Mock()
#     mocker.patch.dict('sys.modules', {'roboy_parlai': import_mock})
#     wildtalk_mock = mocker.patch('roboy_parlai.wildtalk', return_value='test')
#     import ravestate_wildtalk
#     test_dict = {raw_in: 'test'}
#     ravestate_wildtalk.wildtalk_state(test_dict)
#     assert test_dict[raw_out] == 'test'
