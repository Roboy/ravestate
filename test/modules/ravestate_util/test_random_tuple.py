import pytest

from ravestate_util.random_tuple import RandomTuple

TEST_NUMBER = 10


@pytest.fixture
def default_tuple():
    return 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10


@pytest.fixture
def under_test(default_tuple):
    return RandomTuple(default_tuple)


def test_get_random(under_test):
    test_list = []
    for _ in range(TEST_NUMBER):
        test_list.append(under_test.get_random())
    assert len(set(test_list)) > 1
