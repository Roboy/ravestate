
import sys
import os

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")
from ravestate.context import Context
from ravestate_ui import service

ctx = Context(*sys.argv[1:])
ctx.run()
service.advertise(ctx=ctx)
ctx.shutdown()
