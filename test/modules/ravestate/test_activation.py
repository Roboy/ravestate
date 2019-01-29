from ravestate.testfixtures import *
from ravestate.spike import Spike


def test_specificity(activation_fixture):
    assert activation_fixture.specificity() == 1


def test_acquire_spike(activation_fixture):
    assert activation_fixture.acquire(Spike(sig=DEFAULT_PROPERTY_CHANGED, consumable_resources={DEFAULT_PROPERTY_ID}))


def test_acquire_spike_mismatches(activation_fixture):
    assert not activation_fixture.acquire(Spike(sig='x', consumable_resources={DEFAULT_PROPERTY_ID}))


def test_multiple_activation(state_fixture, context_with_property_fixture):
    sa1 = Activation(state_fixture, context_with_property_fixture)
    assert sa1.acquire(
        Spike(sig=DEFAULT_PROPERTY_CHANGED, consumable_resources={DEFAULT_PROPERTY_ID}))
    assert not sa1.acquire(Spike(sig='x', consumable_resources={DEFAULT_PROPERTY_ID}))
    sa2 = Activation(state_fixture, context_with_property_fixture)
    assert sa2.acquire(
        Spike(sig=DEFAULT_PROPERTY_CHANGED, consumable_resources={DEFAULT_PROPERTY_ID}))
    assert not sa2.acquire(Spike(sig='x', consumable_resources={DEFAULT_PROPERTY_ID}))


def test_resources_fallback(activation_fixture_fallback):
    assert activation_fixture_fallback.resources() \
           == {activation_fixture_fallback.state_to_activate.consumable.id()}


# TODO: Add tests for update
# def test_run(activation_fixture):
#     result = activation_fixture.run()
#     assert isinstance(result, Thread)
