import json


class QAParser:
    """
    Getting values from a JSON file
    Parses files containing predefined questions and answers
    Expects the following input pattern:
    {
        "INTENT": {
            "Q": [
                "Question phrasing 1",
                "Question phrasing 2",
                "Question phrasing 3"
            ],
            "A": {
                "SUCCESS": [
                    "Possible answer on success 1",
                    "Possible answer on success 2"
                ],
                "FAILURE": [
                    "Possible answer on failure"
                ]
            }
            "FUP": {
                "Q": [
                    "Possible follow up question"
                ],
                "A": [
                    "Possible follow up answer"
                ]
            }
        }
    }

    See more examples in resources / sentences
    """
    qa_values = None
    questions = {}
    successful_answers = {}
    failure_answers = {}
    followup_questions = {}
    followup_answers = {}

    def __init__(self, path):
        with open(path, 'r') as qa_json:
            data = qa_json.read().replace('\n', '')
            qa_values = json.loads(data)

        self.qa_values = qa_values
        self.parse(qa_values)

    def parse(self, json_string):
        for intent in json_string:
            for attribute, value in json_string[intent].items():
                if attribute == 'Q':
                    self.questions.update({intent: value})
                if attribute == 'A':
                    self.successful_answers.update({intent: value.get('SUCCESS')})
                    self.failure_answers.update({intent: value.get('FAILURE')})
                if attribute == 'FUP':
                    self.followup_questions.update({intent: value.get('Q')})
                    self.followup_answers.update({intent: value.get('A')})

    def get_qa_values(self, intent=None):
        if intent is not None:
            return self.qa_values[intent]
        return self.qa_values

    def get_questions(self, intent=None):
        if intent is not None:
            return self.questions.get(intent)
        return self.questions

    def get_success_answers(self, intent=None):
        if intent is not None:
            return self.successful_answers.get(intent)
        return self.successful_answers

    def get_failure_answers(self, intent=None):
        if intent is not None:
            return self.failure_answers.get(intent)
        return self.failure_answers

    def get_followup_questions(self, intent=None):
        if intent is not None:
            return self.followup_questions.get(intent)
        return self.followup_questions

    def get_followup_answers(self, intent=None):
        if intent is not None:
            return self.failure_answers.get(intent)
        return self.failure_answers