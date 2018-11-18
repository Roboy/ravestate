
import sys
from os.path import join, dirname, realpath
sys.path.append(join(dirname(realpath(__file__)), "modules"))

from ravestate.context import Context
from ravestate_ui import service


ctx = Context(*sys.argv[1:])
ctx.run()
service.advertise(ctx=ctx)
ctx.shutdown()
