from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
from ravestate.state import state
from ravestate.constraint import s

from std_msgs.msg import String

from reggol import get_logger
logger = get_logger(__name__)

with Module(name="ad_demo") as mod:

    @state(cond=s("nlp:triples:changed"),
           read="nlp:triples")
    def pickup_requested(ctx: ContextWrapper):
        # recognise trigger phrase
        # signals go to AD


    @state()
    def arrived_at_pickup_point(ctx: ContextWrapper):
        # processes signal from AD
        # voices: "Hop on!"
        pass


    @state()
    def arrived_at_drop_off_point(ctx: ContextWrapper):
        # processes signal from AD
        # Voices. "We have arrived"
        pass


    @state(cond=s("stalker:recognized_face:changed"))
    def lennart_recognized(ctx: ContextWrapper):
        # check if it is lennart
        # greets lennart
        # triggers handshake: topic ReplayTrajectory which takes a string as a name
        pass
