# Interface of state activation towards signal instance
# Signal instance interface towards state activation

from typing import Set


class ISignalInstance:

    pass


class IStateActivation:

    def write_props(self) -> Set[str]:
        pass

    def wiped(self, sigi: ISignalInstance) -> None:
        pass
