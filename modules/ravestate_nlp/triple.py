from spacy.tokens import Token
from typing import Optional, Set, Union, Tuple
from ravestate_nlp.question_word import QuestionWord


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

    def to_tuple(self) -> Tuple[str, str, str]:
        sub = ""
        pre = ""
        obj = ""
        if self._subject and not self._subject.is_space:
            sub = self._subject.text
        if self._predicate and not self._predicate.is_space:
            pre = self._predicate.lemma_
        if self._predicate_aux and not self._predicate_aux.is_space:
            pre += " " + self._predicate_aux.lemma_
        if self._object and not self._object.is_space:
            obj = self._object.text
        return sub, pre, obj

    def match_either_lemma(self, pred: Optional[Set[str]] = None, subj: Optional[Set[str]] = None, obj: Optional[Set[str]] = None):
        if pred and (self._predicate.lemma_ in pred or self._predicate_aux.lemma_ in pred):
            return True
        if subj and self._subject.lemma_ in subj:
            return True
        if obj and self._object.lemma_ in obj:
            return True
        return False

    def is_question(self, question_word: Optional[str] = None):
        if question_word:
            return self.match_either_lemma(obj={question_word}, subj={question_word})
        else:
            return isinstance(self._subject, QuestionWord) or isinstance(self._object, QuestionWord)

    def ensure_notnull(self, empty_token):
        # do not allow empty entries in triple
        if not self._subject:
            self._subject = empty_token
        if not self._predicate:
            self._predicate = empty_token
        if not self._predicate_aux:
            self._predicate_aux = empty_token
        if not self._object:
            self._object = empty_token

    def __eq__(self, other) -> bool:
        if isinstance(other, Triple):
            return str(self).lower() == str(other).lower()
        elif isinstance(other, tuple) or isinstance(other, list):
            if len(other) == 3:
                return \
                    str(other[0]).lower() == self._subject.text.lower() and \
                    str(other[1]).lower() == self._predicate.lemma_.lower() and \
                    str(other[2]).lower() == self._object.lemma_.lower()
        return False

    def __str__(self):
        return ":".join(self.to_tuple())

    def __repr__(self):
        return f"<Triple object {str(self)}>"
