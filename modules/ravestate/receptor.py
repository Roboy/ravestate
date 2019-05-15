# Ravestate receptor state decorator
from ravestate.property import Property
from ravestate.state import State
from ravestate.wrappers import ContextWrapper
from ravestate.context import Context
from ravestate.activation import Activation
from typing import Union, Set, Tuple


def receptor(*, ctx_wrap: Union[ContextWrapper, Context], write: Union[str, Property, Tuple[Union[str, Property]]]):
    """
    A receptor is a special state which can be invoked from outside,
     to push values into the context.

    * `ctx_wrap`: A context wrapper as is always given
     into the state functions as their first argument.

    * `write`: The property, or tuple of properties, which are going to be written.

    _Example:_
    ```python
    # Module that outputs "Don't Panic" one minute after startup
    # Because a receptor is used, the OUTPUT_PROPERTY is not blocked the whole time
    with Module(name="my_module"):
        @state(cond=startup())
        def after_startup(context):
            @receptor(ctx_wrap=context, write=OUTPUT_PROPERTY)
            def dont_panic(ctx_write):
                ctx_write[OUTPUT_PROPERTY] = "Don't Panic"

            sleep(60)
            dont_panic()
    ```
    """

    def receptor_decorator(action):
        nonlocal ctx_wrap, write
        if isinstance(ctx_wrap, ContextWrapper):
            ctx = ctx_wrap.ctx
        else:
            ctx = ctx_wrap
        receptor_state = State(
            write=write,
            read=(),
            signal=None,
            cond=None,
            action=action,
            is_receptor=True)

        def receptor_function(*args, **kwargs):
            nonlocal receptor_state, ctx
            Activation(st=receptor_state, ctx=ctx).run(*args, **kwargs)

        return receptor_function

    return receptor_decorator
