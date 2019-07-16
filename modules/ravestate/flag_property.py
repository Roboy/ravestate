# Ravestate flag property classes

from typing import Dict, List, Generator
from ravestate.constraint import Signal
from ravestate.property import Property

from reggol import get_logger
logger = get_logger(__name__)


class FlagProperty(Property):
    
    def __init__(
            self, *,
            name="",
            allow_read=True,
            allow_write=True,
            allow_push=True,
            allow_pop=True,
            default_value=None,
            always_signal_changed=False,
            wipe_on_changed=True):

        self.true_signal = Signal(f"{name}:true")
        self.false_signal = Signal(f"{name}:false")

        super().__init__(
            name=name,
            allow_read=allow_read,
            allow_write=allow_write,
            allow_push=allow_push,
            allow_pop=allow_pop,
            default_value=default_value,
            always_signal_changed=always_signal_changed,
            wipe_on_changed=wipe_on_changed)

    def clone(self):
        result = FlagProperty(
            name=self.name,
            allow_read=self.allow_read,
            allow_write=self.allow_write,
            allow_push=self.allow_push,
            allow_pop=self.allow_pop,
            default_value=self.value,
            always_signal_changed=self.always_signal_changed,
            wipe_on_changed=self.wipe_on_changed)
        result.set_parent_path(self.parent_path)
        return result

    def true(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when it is a flag-property and #self.value is set to True.
        """
        return self.true_signal

    def false(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when it is a flag-property and #self.value is set to False.
        """
        return self.false_signal

    def signals(self) -> Generator[Signal, None, None]:
        """
        Yields all signals that may be emitted because of
         this property, given it's write/push/pop permissions.
        """
        yield from super().signals()
        yield self.true_signal
        yield self.false_signal



