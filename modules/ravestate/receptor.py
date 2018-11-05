# Ravestate receptor state decorator

from ravestate.state import State
from ravestate.wrappers import ContextWrapper
from ravestate.activation import StateActivation


def receptor(*, ctx_wrap: ContextWrapper, write):
    """
    A receptor is a special state which can be invoked from outside,
     to push values into the context.
    :param ctx_wrap: A context wrapper as is always given
     into the state functions as their first argument.
    :param write: The property, or tuple of properties, which are going to be written.
    """

    def receptor_decorator(action):
        nonlocal ctx_wrap, write
        ctx = ctx_wrap.ctx
        receptor_state = State(
            write=write,
            read=(),
            signal=None,
            triggers=(),
            action=action,
            is_receptor=True)

        def receptor_function(*args, **kwargs):
            nonlocal receptor_state, ctx
            activation = StateActivation(st=receptor_state, ctx=ctx)
            act_thread = activation.run(args, kwargs)
            act_thread.start()

        return receptor_function

    return receptor_decorator