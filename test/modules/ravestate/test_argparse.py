import pytest
from testfixtures import LogCapture
from os.path import join, dirname, realpath

from ravestate.argparse import handle_args


def test_argparse_empty():
    modules_to_import, config_value_overrides, yaml_file_paths = handle_args()
    assert len(modules_to_import) == 0
    assert len(config_value_overrides) == 0
    assert len(yaml_file_paths) == 0


def test_argparse_simple():
    modules_to_import, config_value_overrides, yaml_file_paths = handle_args("test")
    assert len(modules_to_import) == 1
    assert len(config_value_overrides) == 0
    assert len(yaml_file_paths) == 0
    assert "test" in modules_to_import


def test_argparse_define():
    with LogCapture() as log_capture:
        modules_to_import, config_value_overrides, yaml_file_paths = handle_args(
            "test",
            "-d", "x",
            "-d", "x", "y", "z",
            "-d", "x", "y", "1", "2")
        log_capture.check(('root', 'ERROR', "Not enough values for -d argument: expecting 3, got 1!"))
        assert len(modules_to_import) == 1
        assert len(config_value_overrides) == 2
        assert len(yaml_file_paths) == 0
        assert "test" in modules_to_import
        assert ("x", "y", "z") in config_value_overrides
        assert ("x", "y", ("1", "2")) in config_value_overrides


def test_argparse_yamlpath():
    modules_to_import, config_value_overrides, yaml_file_paths = handle_args(
        "-f", "test1.yml",
        "-f", "test2.yml")
    assert len(modules_to_import) == 0
    assert len(config_value_overrides) == 0
    assert len(yaml_file_paths) == 2
