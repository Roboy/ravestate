from spacy.tokens import Token
from typing import Optional, Set, Union
from ravestate_nlp.question_words import QuestionWord


class Triple:

    _subject: Union[Token, QuestionWord]
    _predicate: Token
    _predicate_aux: Token
    _object: Union[Token, QuestionWord]

    def __init__(self, subject: Token = None, predicate: Token = None, predicate_aux: Token=None, object: Token = None):
        self.set_subject(subject)
        self.set_predicate(predicate)
        self.set_predicate_aux(predicate_aux)
        self.set_object(object)

    def set_subject(self, subject: Union[Token, QuestionWord]):
        self._subject = subject

    def set_predicate(self, predicate: Token):
        self._predicate = predicate
    
    def set_predicate_aux(self, predicate_aux: Token):
        self._predicate_aux = predicate_aux

    def set_object(self, object: Union[Token, QuestionWord]):
        self._object = object

    def get_subject(self) -> Union[Token, QuestionWord]:
        return self._subject

    def get_predicate(self) -> Token:
        return self._predicate

    def get_predicate_aux(self) -> Token:
        return self._predicate_aux

    def get_object(self) -> Union[Token, QuestionWord]:
        return self._object

    def to_tuple(self) -> tuple:
        return self._subject.text, self._predicate.text, self._predicate_aux.text, self._object.text

    def match_either_lemma(self, pred: Optional[Set[str]] = None, subj: Optional[Set[str]] = None, obj: Optional[Set[str]] = None):
        if pred and (self._predicate.lemma_ in pred or self._predicate_aux.lemma_ in pred):
            return True
        if subj:
            if isinstance(self._subject, QuestionWord):
                if self._subject.text in subj:
                    return True
            elif self._subject.lemma_ in subj:
                return True
        if obj:
            if isinstance(self._object, QuestionWord):
                if self._object.text in obj:
                    return True
            elif self._object.lemma_ in obj:
                return True
        return False

    def is_question(self, question_word: Optional[str] = None):
        if question_word:
            return self.match_either_lemma(obj={question_word}, subj={question_word})
        else:
            return isinstance(self._subject, QuestionWord) or isinstance(self._object, QuestionWord)

    def __eq__(self, other) -> bool:
        if isinstance(other, self.__class__):
            if self._subject.text != other.get_subject().text:
                return False
            if self._predicate.text != other.get_predicate().text:
                return False
            if self._predicate_aux.text != other.get_predicate_aux().text:
                return False
            if self._object.text != other.get_object().text:
                return False
            return True
        if isinstance(other, tuple) and isinstance(other[0], str) and len(other) == 3:
            if self._subject.text.lower() != other[0].lower():
                return False
            if self._predicate.lemma_.lower() != other[1].lower():
                return False
            if self._predicate_aux.lemma_.lower() != other[1].lower():
                return False
            if self._object.text.lower() != other[2].lower():
                return False
            return True
        return False

    def __str__(self):
        sub = ''
        pred = ''
        pred_aux = ''
        obj = ''
        space = ''
        if self._subject:
            sub = self._subject.text
        if self._predicate:
            pred = self._predicate.lemma_
        if self._predicate_aux and not self._predicate_aux.text == " ":
            pred_aux = self._predicate_aux.lemma_
            space = ' '
        if self._object:
            obj = self._object.text
        return f'{sub}:{pred}{space}{pred_aux}:{obj}'.lower()

    def __repr__(self):
        sub = ''
        pred = ''
        pred_aux = ''
        obj = ''
        space = ''
        if self._subject:
            sub = self._subject.text
        if self._predicate:
            pred = self._predicate.lemma_
        if self._predicate_aux and not self._predicate_aux.text == " ":
            pred_aux = self._predicate_aux.lemma_
            space = ' '
        if self._object:
            obj = self._object.text
        return f'<Triple object {sub}:{pred}{space}{pred_aux}:{obj}>'
