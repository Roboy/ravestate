
import sys

from ravestate.context import Context
from ravestate_ui import service


ctx = Context(*sys.argv[1:])
ctx.run()
service.advertise(ctx=ctx)
ctx.shutdown()
