from ravestate.testfixtures import *


def test_register(mocker):
    from ravestate import registry
    with mocker.patch('ravestate.registry.register'):
        import ravestate_roboyqa
        registry.register.assert_called_with(
            name="roboyqa",
            states=(ravestate_roboyqa.roboyqa,),
            config={ravestate_roboyqa.ROBOY_NODE_CONF_KEY: 356})


def test_roboyqa(mocker, context_fixture, triple_fixture):
    mocker.patch.object(context_fixture, 'conf', will_return='test')
    context_fixture._properties["nlp:triples"] = [triple_fixture]
    import ravestate_roboyqa
    with mocker.patch('ravestate_ontology.get_session'):
        ravestate_roboyqa.roboyqa(context_fixture)
