import ravestate as rs

# We want to write some text output, so we
# need the raw:out context property from ravestate_rawio.
import ravestate_rawio as rawio
from typing import Dict

from reggol import get_logger
logger = get_logger(__name__)

import time

stickers: Dict = {"explosion": "CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag",
                    "dance": "CAADAgADrwAD5dCAELOikjem6GCQAg",
                    "kenny": "CAADAgADkAAD5dCAEGMfygavvZSZAg",
                    "beer": "CAADAgADKQAD5dCAEFX3hCMAAfM_awI",
                    "happy": "CAADAgADRgAD5dCAEJV_o50ekE5HAg",
                    "hearts": "CAADAgADSQAD5dCAEN9n0g-x5va8Ag",
                    "roboyrick": "CAADAgADjwAD5dCAEA26JXGqLEGhAg",
                    "sunglasses": "CAADAgADLwAD5dCAENTFuFLbW8-XAg",
                    "rainbow": "CAADAgADVQAD5dCAEHTBjm9cSbBTAg",
                    "pickle": "CAADAgADwgAD5dCAEKjfQCRuUDfYAg"}

# Ravestate applications should always be wrapped in a Module.
# This allows easier scoping, and enables separation of concerns
# beyond states.
with rs.Module(name="stickerbot") as mod:

    # Create an application state which reacts to the `:startup` signal,
    # and writes a string to raw:out. Note: State functions are
    # always run asynchronously!
    @rs.state(cond=rs.sig_startup, write=rawio.prop_out)
    def hello(ctx: rs.ContextWrapper):
        logger.info("stickerbot active")
        # return rs.Resign()
        ctx[rawio.prop_out] = "sticker:CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag"
