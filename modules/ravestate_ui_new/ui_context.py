from ravestate.context import Context
from ravestate.spike import Spike
from ravestate.activation import Activation
from typing import Any, Tuple, List

import websockets
import asyncio
from threading import Thread, Lock

from reggol import get_logger
logger = get_logger(__name__)


class UIContext(Context):

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        super().__init__(*arguments, runtime_overrides=runtime_overrides)
        self.msgs_sem = asyncio.Semaphore(value=0)
        self.msgs_lock = Lock()

        def run_serve_async():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            async_serve = websockets.serve(self.serve_events, "localhost", 5000)
            loop.run_until_complete(async_serve)
        Thread(target=run_serve_async).start()

    async def serve_events(self, ws, path):
        await self.msgs_sem.acquire()
        with self.msgs_lock:
            await ws.send("Test")

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
            self._spikes_per_signal[signal].add(new_spike)

    def _state_activated(self, act: Activation):
        self._activations_per_state[act.state_to_activate].discard(act)
        self.msgs_sem.release()
