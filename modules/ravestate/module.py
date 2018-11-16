# Ravestate module class

from typing import List, Dict, Any, Optional
from ravestate.property import PropertyBase
from ravestate.state import State


class Module:
    """
    Atomic class, which encapsulates a named set of states, properties and config entries,
    which form a coherent bundle.
    """

    def __init__(self, *, name: str,
                 props: Optional[List[PropertyBase]]=(),
                 states: Optional[List[State]]=(),
                 config: Optional[Dict[str, Any]]=None):

        if not isinstance(props, tuple):
            props = (props,)
        if not isinstance(states, tuple):
            props = (states,)
        if not config:
            config = {}
        self.name = name
        self.props = props
        self.states = states
        self.conf = config


