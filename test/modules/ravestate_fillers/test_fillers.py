from ravestate.testfixtures import *

import ravestate_rawio
import ravestate_fillers
import ravestate_verbaliser
from ravestate_fillers import impatient_fillers


def test_fillers():
    ctx = Context("rawio", "idle", "verbaliser", "fillers")
    assert ctx["verbaliser:intent"].read() == ""
    ctx.emit(s("idle:impatient"))
    ctx.run_once()
    assert impatient_fillers.wait()
    assert ctx["verbaliser:intent"].read() == "fillers"
