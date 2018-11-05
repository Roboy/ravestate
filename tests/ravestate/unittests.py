import unittest

import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../../modules")

from ravestate.context import Context


class Sample_unit_tests(unittest.TestCase):
    def test_hello(self):
        print("Hello Travis")
        assert 1 == 1

    def test_init(self):
        ctx = Context()
        assert ctx
