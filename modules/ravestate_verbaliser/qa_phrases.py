from typing import List


def check_if_list(value):
    if isinstance(value, List):
        return value
    raise AttributeError("Value is not a list: " + str(value))


class QAPhrases:
    """
    Getting values from a yml file
    Parses files containing predefined questions and answers
    Expects the following input pattern:
    ---
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
    questions: List[str] = None
    successful_answers: List[str] = None
    failure_answers: List[str] = None
    followup_questions: List[str] = None
    followup_answers: List[str] = None

    def __init__(self, yaml_data: dict):
        """
        Initializes the lists for the different QA-elements with the data in the yaml_data-dict

        :param yaml_data: dict of one section of a yml file with the structure of the above example
            converted to a dict by yaml.safe_load_all(path)
        """

        for key in yaml_data:
            if key == 'Q':
                self.questions = check_if_list(yaml_data[key])
            elif key == 'A':
                for subkey in yaml_data[key]:
                    if subkey == 'SUCCESS':
                        self.successful_answers = check_if_list(yaml_data[key][subkey])
                    elif subkey == 'FAILURE':
                        self.failure_answers = check_if_list(yaml_data[key][subkey])
            elif key == 'FUP':
                for subkey in yaml_data[key]:
                    if subkey == 'Q':
                        self.followup_questions = check_if_list(yaml_data[key][subkey])
                    elif subkey == 'A':
                        self.followup_answers = check_if_list(yaml_data[key][subkey])
