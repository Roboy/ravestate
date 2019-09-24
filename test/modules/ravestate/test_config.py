from ravestate.testfixtures import *
from os.path import join, dirname, realpath

from ravestate.config import Configuration
from ravestate.module import Module


@pytest.fixture
def config_file_paths():
    return [
        join(dirname(realpath(__file__)), filename)
        for filename in
        ("test_config_file1.yml", "test_config_file2.yml")
    ]

@pytest.fixture
def under_test():
    return Configuration([])


def test_read_config_1(under_test: Configuration, config_file_paths):
    assert len(under_test.parsed_config_per_module) == 0
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test.read(config_file_paths[0])
        log_capture.check(f"Skipping invalid entry for config file {config_file_paths[0]}.")
        assert len(under_test.parsed_config_per_module) == 2
        assert under_test.parsed_config_per_module["foo"]["a"] is True
        assert under_test.parsed_config_per_module["poo"]["c"] == 1
        assert under_test.parsed_config_per_module["poo"]["d"] == "hehe"


def test_read_config_2(under_test: Configuration, config_file_paths):
    under_test.read(config_file_paths[0])
    under_test.read(config_file_paths[1])
    assert under_test.parsed_config_per_module["foo"]["a"] is False
    assert under_test.parsed_config_per_module["poo"]["c"] == 1
    assert under_test.parsed_config_per_module["poo"]["d"] == "hehe"


def test_read_config_3(config_file_paths):
    under_test = Configuration(paths=config_file_paths)
    assert under_test.parsed_config_per_module["foo"]["a"] is False
    assert under_test.parsed_config_per_module["poo"]["c"] == 1
    assert under_test.parsed_config_per_module["poo"]["d"] == "hehe"


def test_get_conf(under_test: Configuration):
    under_test.add_conf(Module(name="get_conf", config={"a": False, "b": 42}))
    config_dict = under_test.get_conf(module_name="get_conf")
    assert len(config_dict) == 2
    assert config_dict["a"] is False
    assert config_dict["b"] == 42


def test_apply_config(under_test: Configuration, config_file_paths):
    under_test.read(config_file_paths[0])
    under_test.read(config_file_paths[1])
    foo_module = Module(name="foo", config={
        "a": "hehe",
        "b": "hoho",
        "c": "lol"
    })
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test.add_conf(foo_module)
        log_capture.check(f"Config entry for foo.a has conflicting type bool (should be str).")
        assert under_test.get("foo", "a") == False
        assert under_test.get("foo", "b") == "haha"
        assert under_test.get("foo", "c") == "lol"
        under_test.add_conf(foo_module)
        log_capture.check_present(f"add_conf called repeatedly for module name foo!")


def test_list_convert(under_test: Configuration, config_file_paths):
    under_test.read(config_file_paths[0])
    under_test.read(config_file_paths[1])
    under_test.add_conf(Module(name="foo", config={
        "a": "hehe",
        "b": "hoho",
        "c": "lol",
        "d": [],
        "e": [1]
    }))
    under_test.set("foo", "d", "value")
    value = under_test.get("foo", "d")
    assert isinstance(value, list) and value[0] == "value"
    under_test.set("foo", "e", "30")
    value = under_test.get("foo", "e")
    assert isinstance(value, list) and value[0] == 30
    under_test.set("foo", "e", [True])
    value = under_test.get("foo", "e")
    assert isinstance(value, list) and value[0] is True


def test_unknown_module(under_test: Configuration):
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test.get_conf(DEFAULT_MODULE_NAME)
        log_capture.check(f"Bad request for unknown module config {DEFAULT_MODULE_NAME}!")

        under_test.set(module_name=DEFAULT_MODULE_NAME, key="a", value=True)
        log_capture.check_present(f"Attempt to run set() for unknown modname {DEFAULT_MODULE_NAME}!")

        under_test.add_conf(Module(name=DEFAULT_MODULE_NAME, config={}))
        under_test.set(module_name=DEFAULT_MODULE_NAME, key="a", value=True)
        log_capture.check_present(f"Not setting unknown conf key a for module {DEFAULT_MODULE_NAME}.")


def test_config_write(mocker, under_test: Configuration):
    under_test.add_conf(Module(name=DEFAULT_MODULE_NAME, config={"a": "value_A"}))
    m = mocker.mock_open()
    mocker.patch('builtins.open', m)
    under_test.write(path="config_mock.yml")

    m.assert_called_once_with('config_mock.yml', mode='w')
    handle = m()
    expected_calls = ['config', ':', '\n', '  ', 'a', ':', ' ', 'value_A', '\n',
                      'module', ':', ' ', 'module', '\n']
    handle.write.assert_has_calls([mocker.call(exp_call) for exp_call in expected_calls])


def test_apply_parsed_unknown_config(under_test: Configuration, config_file_paths):
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test.read(config_file_paths[1])
        under_test._apply_parsed_config(module_name='foo')
        log_capture.check(f"Attempt to run _apply_parsed_config for unknown modname foo!")
