from ravestate.context import Context
import sys

ctx = Context(*sys.argv[1:])
ctx.run()

# TODO: Make sure that UI is cleanly integrated as a ravestate module.
# from ravestate_ui import service
# service.advertise()