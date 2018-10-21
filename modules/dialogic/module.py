# Dialogic module class


class Module:
    """
    Atomic class, which encapsulates a named set of states, properties and config entries,
    which form a coherent bundle.
    """

    def __init__(self, *, name: str, props=(), states=(), config={}):
        if not isinstance(props, tuple):
            props = (props,)
        if not isinstance(states, tuple):
            props = (states,)
        self.name = name
        self.props = props
        self.states = states

