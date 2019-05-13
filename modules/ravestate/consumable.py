# Dummy resource which allowed CausalGroups to track acquisitions
#  for states that don't have any write-props.

from ravestate.property import Property

from reggol import get_logger
logger = get_logger(__name__)


class Consumable(Property):
    """
    Dummy resource which allows CausalGroups to track acquisitions
     for states that don't have any write-props.
    """

    def __init__(self, name):
        super().__init__(name=name, allow_read=False, allow_write=False, allow_push=False, allow_pop=False)
