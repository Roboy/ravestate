from ravestate.context import Context
import sys

ctx = Context(*sys.argv[1:])
ctx.run()
