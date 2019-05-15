from ravestate.context import Context
from typing import Any, Tuple, List
from ravestate_ui import service
from ravestate.spike import Spike
from ravestate.activation import Activation

from reggol import get_logger
logger = get_logger(__name__)


class UIContext(Context):

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        super().__init__(*arguments, runtime_overrides=runtime_overrides)
        service.fillme(self._activations_per_state, self._properties)
        service.advertise()

    def emit(self, signal, parents=None, wipe: bool=False, payload=None) -> None:
        # Copy instead of calling super to obtain reference to created spike.
        if wipe:
            self.wipe(signal)
        with self._lock:
            new_spike = Spike(
                sig=signal.id(),
                parents=parents,
                consumable_resources=set(self._properties.keys()),
                payload=payload)
            logger.debug(f"Emitting {new_spike}")
            service.spike(new_spike.id())
            self._spikes_per_signal[signal].add(new_spike)

    def _state_activated(self, act: Activation):
        service.activate(act.state_to_activate.name)
        self._activations_per_state[act.state_to_activate].discard(act)
