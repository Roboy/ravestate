from ravestate.property import PropertyBase
from ravestate.module import Module


with Module(name="interloc"):

    # TODO: Make interloc:all a special property type, that only accepts ScientioNodeProperty as children
    all = PropertyBase(name="all", allow_read=True, allow_write=False, allow_push=True, allow_pop=True)
