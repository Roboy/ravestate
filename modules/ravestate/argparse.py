# Ravestate argument parser

from argparse import ArgumentParser
from argparse import RawDescriptionHelpFormatter
from argparse import Action
from typing import List, Tuple, Any

from reggol import get_logger, help_string
logger = get_logger(__name__)


def handle_args(*args) -> Tuple[List[str], List[Tuple[str, str, Any]], List[str]]:
    """
    Runs an argument parser for the given args. Returns modules-to-load,
    config value-overrides and config file-pathes.
     Note: If the arguments are ill-formatted, or the -h argument is passed,
    help will be printed to the console and the program will abort.
    :param args: Argument list which will be fed into argparse.parse_args.
    :return: A Tuple with three items:
    1.) A list of module names which should be imported.
    2.) A list of tuples, where each tuple is a module name, a config key name, and a value.
    3.) A list of yaml file paths.
    """
    modules_to_import: List[str] = []
    config_value_overrides: List[Tuple[str, str, Any]] = []
    yaml_file_paths: List[str] = []

    argdef = ArgumentParser(
        description="Run a reactive state machine! It's weird, you'll like it.",
        formatter_class=RawDescriptionHelpFormatter,
        epilog=f"""
{help_string()}

usage:
  > rasta ravestate_facerec ravestate_hello_world
    Import two python modules and run a context.

  > rasta \\
      -d core import ravestate_facerec ravestate_hello_world \\
      -f generic.yml \\
      -f user.yml
    Import two python modules and run a context,
    with additional config entries loaded from generic.yml
    and user.yml, where entries in user.yml overload entries
    in generic.yml.
"""
    )

    class ModulesAction(Action):
        def __call__(self, parser, args, values, option_string=None):
            nonlocal modules_to_import
            if isinstance(values, str):
                values = [values]
            modules_to_import += values

    class OverrideAction(Action):
        def __call__(self, parser, args, values, option_string=None):
            nonlocal config_value_overrides
            if isinstance(values, str):
                values = [values]
            if len(values) < 3:
                logger.error(f"Not enough values for -d argument: expecting 3, got {len(values)}!")
                return
            if len(values) > 3:
                values = (values[0], values[1], tuple(values[2:]))
            else:
                values = tuple(values)
            config_value_overrides += [values]

    class ConfigFileAction(Action):
        def __call__(self, parser, args, values, option_string=None):
            nonlocal yaml_file_paths
            if isinstance(values, str):
                values = [values]
            yaml_file_paths += values

    argdef.add_argument(
        "modules", nargs="*", type=str, metavar="python-module-name",
        action=ModulesAction,
        help="""
        List ravestate python modules which should be loaded.
        """)

    argdef.add_argument(
        "-d", "--define",
        nargs="+", type=str, metavar=("module key", "values"),
        action=OverrideAction,
        help="""
        Set the value(s) for a certain config entry. If multiple values are given,
        they will be converted to a list. An error will be raised if the given
        module does not exist, or hasn't declared the specified config entry,
        or the given value type (list/scalar) doesn't match the declared.
        """)

    argdef.add_argument(
        "-f", "--config",
        nargs=1, type=str, metavar="yaml-path",
        action=ConfigFileAction,
        help="""
        Add a yaml file path, from which config entries for a certain
        module should be read. The file should contain a `module` key
        which specifies the target module name, and a `config` key which
        is a dictionary of config values.
         Note, that config entries specified in later yaml files override
        entries from earlier files.
        """)
    argdef.add_help = True
    argdef.parse_args(args)

    return modules_to_import, config_value_overrides, yaml_file_paths
