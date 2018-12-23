# Interface for the context towards activation and
#  context/property wrappers (needed to preempt cyclic dependencies).


from ravestate import property
from ravestate import state
from typing import Set
from ravestate.constraint import Signal

class IContext:

    def emit(self, signal_name: str):
        pass

    def add_state(self, *, st: state.State):
        pass

    def rm_state(self, *, st: state.State):
        pass

    def add_prop(self, *, prop: property.PropertyBase):
        pass

    def rm_prop(self, *, prop: property.PropertyBase):
        pass

    def shutting_down(self):
        pass

    def shutdown(self):
        pass

    def conf(self, *, mod, key=None):
        pass

    def get_prop(self, key):
        pass

    def predict(self, signals: Set[Signal]) -> int:
        pass
