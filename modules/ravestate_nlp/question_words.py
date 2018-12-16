from spacy.tokens import Token


class QuestionWord:
    _form = 'FORM'
    _choice = 'CHOICE'
    _object = 'OBJECT'
    _reason = 'REASON'
    _time = 'TIME'
    _person = 'PERSON'
    _place = 'PLACE'

    question_pos = 'QUESTION'
    question_words = {
        'where': _place,
        'who': _person,
        'when': _time,
        'why': _reason,
        'what': _object,
        'which': _choice,
        'how': _form
    }

    def __init__(self, token: Token):
        self.text = self.question_words[token.text.lower()]
        self.pos_ = self.question_pos
        self.dep_ = token.dep_
