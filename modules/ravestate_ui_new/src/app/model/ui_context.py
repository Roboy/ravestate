from ravestate.context import Context
from typing import Any, Tuple, List
from ravestate_ui_new.src.app.model import endpoints

from reggol import get_logger
logger = get_logger(__name__)


class UIContext(Context):

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        super().__init__(*arguments, runtime_overrides=runtime_overrides)
        endpoints.advertise()


    def spike_incoming(self):
        """
        DUMMY

        Sends data about a new activated spike to the frontend.
        Triggered whenever a spike is created.
        """
        pass

    def build_ui_tick_update(self):
        """
        DUMMY

        Sends data about all candidate activations and activated activations to the frontend.
        Triggered upon a tick.
        """
        pass

