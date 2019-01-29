from ravestate.testfixtures import *


def test_roboyqa(mocker, context_fixture, triple_fixture):
    mocker.patch.object(context_fixture, 'conf', will_return='test')
    context_fixture._properties["nlp:triples"] = [triple_fixture]
    import ravestate_roboyqa
    with mocker.patch('ravestate_ontology.get_session'):
        ravestate_roboyqa.roboyqa(context_fixture)
