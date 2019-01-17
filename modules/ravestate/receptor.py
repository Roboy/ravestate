# Ravestate receptor state decorator

from ravestate.state import State
from ravestate.wrappers import ContextWrapper
from ravestate.activation import Activation
from typing import Union, Set, Tuple


def receptor(*, ctx_wrap: ContextWrapper, write: Union[str, Tuple[str]]):
    """
    A receptor is a special state which can be invoked from outside,
     to push values into the context.

    * `ctx_wrap`: A context wrapper as is always given
     into the state functions as their first argument.

    * `write`: The property, or tuple of properties, which are going to be written.
    """

    def receptor_decorator(action):
        nonlocal ctx_wrap, write
        ctx = ctx_wrap.ctx
        receptor_state = State(
            write=write,
            read=(),
            signal_name=None,
            cond=None,
            action=action,
            is_receptor=True)

        def receptor_function(*args, **kwargs):
            nonlocal receptor_state, ctx
            Activation(st=receptor_state, ctx=ctx).run(*args, **kwargs)

        return receptor_function

    return receptor_decorator
