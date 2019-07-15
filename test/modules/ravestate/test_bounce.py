import unittest
import time
from ravestate.debounce import debounce


class TestDebounce(unittest.TestCase):

    @debounce(10)
    def increment(self):
        """ Simple function that
            increments a counter when
            called, used to test the
            debounce function decorator """
        self.count += 1

    def setUp(self):
        self.count = 0

    def test_debounce(self):
        """ Test that the increment
            function is being debounced.
            The counter should only be incremented
            once 10 seconds after the last call
            to the function """
        self.assertTrue(self.count == 0)
        self.increment()
        self.increment()
        time.sleep(9)
        self.assertTrue(self.count == 0)
        self.increment()
        self.increment()
        self.increment()
        self.increment()
        self.assertTrue(self.count == 0)
        time.sleep(10)
        self.assertTrue(self.count == 1)

if __name__ == '__main__':
    unittest.main()