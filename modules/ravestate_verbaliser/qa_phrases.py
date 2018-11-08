from typing import List


class QAPhrases:
    """
    Getting values from a yml file
    Parses files containing predefined questions and answers
    Expects the following input pattern:
    type: qa
    name: "INTENT"
    Q:
    - "Question phrasing 1"
    - "Question phrasing 2"
    - "Question phrasing 3"
    A:
      SUCCESS:
      - "Possible answer on success 1"
      - "Possible answer on success 2"
      FAILURE:
      - "Possible answer on failure"
    FUP:
      Q:
      - "Possible follow up question"
      A:
      - "Possible follow up answer"

    See more examples in resources / sentences
    """
    questions: List[str] = []
    successful_answers: List[str] = []
    failure_answers: List[str] = []
    followup_questions: List[str] = []
    followup_answers: List[str] = []

    def __init__(self, yaml_data: dict):
        for key in yaml_data:
            if key == 'Q':
                self.questions = yaml_data[key]
            elif key == 'A':
                for subkey in yaml_data[key]:
                    if subkey == 'SUCCESS':
                        self.successful_answers = yaml_data[key][subkey]
                    elif subkey == 'FAILURE':
                        self.failure_answers = yaml_data[key][subkey]
            elif key == 'FUP':
                for subkey in yaml_data[key]:
                    if subkey == 'Q':
                        self.followup_questions = yaml_data[key][subkey]
                    elif subkey == 'A':
                        self.followup_answers = yaml_data[key][subkey]
