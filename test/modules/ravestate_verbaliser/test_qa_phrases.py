import pytest
from ravestate_verbaliser.qa_phrases import QAPhrases

questions = ['question1', 'question2']
successful_answers = ['successful_answer1', 'successful_answer2']
failure_answers = ['failure_answer1', 'failure_answer2']
followup_questions = ['followup_question1', 'followup_question2']
followup_answers = ['followup_answer1', 'followup_answer2']


@pytest.fixture
def qa_dict():
    return {'type': 'qa',
            'name': 'intent',
            'Q': questions,
            'A': {'SUCCESS': successful_answers,
                  'FAILURE': failure_answers},
            'FUP': {'Q': followup_questions,
                    'A': followup_answers}
            }


def test_init(qa_dict):
    qa = QAPhrases(qa_dict)
    assert qa.questions == questions
    assert qa.successful_answers == successful_answers
    assert qa.failure_answers == failure_answers
    assert qa.followup_questions == followup_questions
    assert qa.followup_answers == followup_answers


def test_invalid_attribute_init(qa_dict):
    qa_dict['Q'] = 'not a list'
    with pytest.raises(AttributeError):
        QAPhrases(qa_dict)
