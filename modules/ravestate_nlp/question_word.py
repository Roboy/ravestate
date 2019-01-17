from spacy.tokens import Token


class QuestionWord:
    FORM = 'FORM'
    CHOICE = 'CHOICE'
    OBJECT = 'OBJECT'
    REASON = 'REASON'
    TIME = 'TIME'
    PERSON = 'PERSON'
    PLACE = 'PLACE'

    question_pos = 'QUESTION'
    question_words = {
        'where': PLACE,
        'who': PERSON,
        'when': TIME,
        'why': REASON,
        'what': OBJECT,
        'which': CHOICE,
        'how': FORM
    }

    def __init__(self, token: Token):
        self.text = self.question_words[token.text.lower()]
        self.lemma_ = self.question_words[token.text.lower()]
        self.pos_ = self.question_pos
        self.dep_ = token.dep_
        self.is_space = False
