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
            always_signal_changed=False, ):

        self.name = name
        self.allow_read = allow_read
        self.allow_write = allow_write
        self.allow_push = allow_push
        self.allow_pop = allow_pop
        self.allow_delete = allow_delete
        self.value = default
        self.children: Dict[str, PropertyBase] = dict()
        self._lock = Lock()
        self.parent_path: str = ""
        self.always_signal_changed = always_signal_changed

    def fullname(self):
        return f'{self.parent_path}:{self.name}'

    def set_parent_path(self, path):
        """
        Set the ancestors (including modulename) for a property
        :param path: ancestry in the form of modulename:parent_prop_name (or simply modulename)
        """
        if not self.parent_path:
            self.parent_path = path
        else:
            logger.error(f'Tried to override parent_path of {self.fullname()}')

    def lock(self):
        self._lock.acquire()

    def unlock(self):
        self._lock.release()

    def read(self, child: List[str] = None):
        """
        Read the current property value or the value of children if child-param is given
        :param child: top-down list of child ancestry
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
        Write a new value to the property or to a child property if child-param is given
        :param value: The new value.
        :param child: top-down list of child ancestry
        :return: True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            logger.error(f"Unauthorized write access in property {self.fullname()}!")
            return False
        if child:
            if child[0] in self.children:
                return self.children[child[0]].write(value, child=child[1:])
            else:
                logger.error(f'Tried to write to non-existent child-property {self.fullname()}:{child[0]}')
                return None
        else:
            if self.always_signal_changed or self.value != value:
                self.value = value
                return True
            else:
                return False

    def push(self, child_list: List[str], always_signal_changed: bool = False, default=None):
        """
        Add a child to the property or to children of the property
        :param child_list: top-down list of child ancestry for the new child
        :param always_signal_changed: new child property will set this for always_signal_changed
        :param default: default value for new child property
        :return: True if the push was successful, False otherwise
        """
        if not self.allow_push:
            logger.error(f"Unauthorized push in property {self.fullname()}!")
            return False
        if not child_list:
            logger.error(f'Tried to push empty children List to {self.fullname()}!')
            return False
        direct_child_name = child_list[0]
        if len(child_list) == 1 and direct_child_name in self.children:
            logger.error(f"Tried to add already existing child-property {self.fullname()}:{direct_child_name}")
            return False
        if direct_child_name not in self.children:
            child_prop = PropertyBase(name=direct_child_name,
                                      allow_read=self.allow_read,
                                      allow_write=self.allow_write,
                                      allow_push=self.allow_push,
                                      allow_pop=self.allow_pop,
                                      allow_delete=self.allow_delete,
                                      default=default,
                                      always_signal_changed=always_signal_changed)
            child_prop.set_parent_path(f'{self.fullname()}')
            child_prop.lock()
            if self.allow_read:
                pass  # TODO frozen value for children
            if not self.allow_write:
                child_prop.unlock()

            self.children[direct_child_name] = child_prop
        if len(child_list) > 1:
            return self.children[direct_child_name].push(child_list[1:])
        else:
            return True

    def pop(self, child_list: List[str]):
        """
        Remove a child from the property or from children of the property
        :param child_list: top-down list of child ancestry of the child to be removed
        :return: True if the pop was successful, False otherwise
        """
        if not self.allow_pop:
            logger.error(f"Unauthorized pop in property {self.fullname()}!")
            return False
        if not child_list:
            logger.error(f'Tried to pop empty children List from {self.fullname()}')
            return False
        if len(child_list) > 1:
            return self.children[child_list[0]].pop(child_list[1:])
        elif child_list[0] in self.children:
            popped_child = self.children.pop(child_list[0])
            if self.allow_write:
                popped_child.unlock()
            return True
        else:
            logger.error(f"Tried to remove non-existent child-property {self.fullname()}:{child_list[0]}")
            return False
