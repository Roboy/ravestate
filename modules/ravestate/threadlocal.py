import threading

# Local store used by Module, PropertyBase and State,
#  to facilitate `with Module(...)` enter/exit.
ravestate_thread_local = threading.local()
