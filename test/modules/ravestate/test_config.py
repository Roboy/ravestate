import pytest
from testfixtures import LogCapture
from os.path import join, dirname, realpath

from ravestate.config import Configuration
from ravestate.module import Module
from reggol import strip_prefix


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
        assert under_test.parsed_config_per_module["foo"]["a"] == True
        assert under_test.parsed_config_per_module["poo"]["c"] == 1
        assert under_test.parsed_config_per_module["poo"]["d"] == "hehe"


def test_read_config_2(under_test: Configuration, config_file_paths):
    under_test.read(config_file_paths[0])
    under_test.read(config_file_paths[1])
    assert under_test.parsed_config_per_module["foo"]["a"] == False
    assert under_test.parsed_config_per_module["poo"]["c"] == 1
    assert under_test.parsed_config_per_module["poo"]["d"] == "hehe"


def test_apply_config(under_test: Configuration, config_file_paths):
    under_test.read(config_file_paths[0])
    under_test.read(config_file_paths[1])
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test.add_conf(Module(name="foo", config={
            "a": "hehe",
            "b": "hoho",
            "c": "lol"
        }))
        log_capture.check(f"Config entry for foo.a has conflicting type bool (should be str).")
        assert under_test.get("foo", "a") == False
        assert under_test.get("foo", "b") == "haha"
        assert under_test.get("foo", "c") == "lol"


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
    assert isinstance(value, list) and value[0] == True
