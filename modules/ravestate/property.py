# Ravestate property classes

from threading import Lock
from typing import Dict, List

from reggol import get_logger

logger = get_logger(__name__)


class PropertyBase:
    """
    Base class for context properties. Controls read/write/push/pop/delete permissions,
    property name basic impls. for the property value, parent/child mechanism.
    """

    def __init__(
            self, *,
            name="",
            allow_read=True,
            allow_write=True,
            allow_push=True,
            allow_pop=True,
            allow_delete=True,
            default=None,
            always_signal_changed=False):

        self.name = name
        self.allow_read = allow_read
        self.allow_write = allow_write
        self.allow_push = allow_push
        self.allow_pop = allow_pop
        self.allow_delete = allow_delete
        self.value = default
        self.children: Dict[str, PropertyBase] = dict()
        self._lock = Lock()
        self.module_name = ""
        self.always_signal_changed = always_signal_changed

    def fullname(self):  # TODO names of children
        return f"{self.module_name}:{self.name}"

    def lock(self):
        self._lock.acquire()

    def unlock(self):
        self._lock.release()

    def read(self, child: List[str] = None):
        """
        Read the current property value.
        """
        if not self.allow_read:
            logger.error(f"Unauthorized read access in property {self.fullname()}!")
            return None
        if child:
            if child[0] in self.children:
                return self.children[child[0]].read(child=child[1:])
            else:
                logger.error(f'Tried to read non-existent child-property {self.fullname()}:{child[0]}')
                return None
        else:
            return self.value

    def write(self, value, child: List[str] = None):
        """
        Write a new value to the property.
        :param value: The new value.
        :return: True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            logger.error(f"Unauthorized write access in property {self.name}!")
            return False
        if child:
            # TODO always_signal_changed for children?
            if child[0] in self.children:
                return self.children[child[0]].write(value, child=child[1:])
            else:
                # TODO in grandchild error message it looks like a normal child,
                #  because the grandchild doesn't know about the parents
                logger.error(f'Tried to write to non-existent child-property {self.fullname()}:{child}')
                return None
        else:
            if self.always_signal_changed or self.value != value:
                self.value = value
                return True
            else:
                return False

    def push(self, child_list: List[str]):
        # TODO permission?
        if not child_list:
            logger.error(f'Tried to push empty children List to {self.fullname()}')
            return False
        if len(child_list) > 1:
            if child_list[0] not in self.children:
                self.children[child_list[0]] = PropertyBase(name=child_list[0])
            return self.children[child_list[0]].push(child_list[1:])

        if child_list[0] not in self.children:
            self.children[child_list[0]] = PropertyBase(name=child_list[0])
            return True
        else:
            logger.error(f"Tried to add already existing child-property {self.fullname()}:{child_list}")
            return False

    def pop(self, child_list: List[str]):
        # TODO permission?
        if not child_list:
            logger.error(f'Tried to pop empty children List from {self.fullname()}')
            return False
        if len(child_list) > 1:
            return self.children[child_list[0]].pop(child_list[1:])
        elif child_list[0] in self.children:
            self.children.pop(child_list[0])
            return True
        else:
            logger.error(f"Tried to remove non-existent child-property {self.fullname()}:{child_list[0]}")
            return False
