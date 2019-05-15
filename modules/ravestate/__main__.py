from ravestate.context import Context
import sys

#from hanging_threads import start_monitoring
#monitoring_thread = start_monitoring()

ctx = Context(*sys.argv[1:])
ctx.run()
