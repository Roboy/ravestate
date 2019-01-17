# Ravestate module class

from typing import Dict, Any, Tuple
from ravestate.property import PropertyBase
from ravestate.state import State


class Module:
    """
    Atomic class, which encapsulates a named set of states, properties and config entries,
    which form a coherent bundle.
    """

    def __init__(self, *, name: str,
                 props: Tuple[PropertyBase]=(),
                 states: Tuple[State]=(),
                 config: Dict[str, Any]=None):

        if not isinstance(props, tuple):
            props = (props,)
        if not isinstance(states, tuple):
            states = (states,)
        if not config:
            config = {}

        self.name = name
        self.props = props
        self.states = states
        self.conf = config

        for prop in props:
            prop.set_parent_path(name)
        for st in states:
            st.module_name = name
