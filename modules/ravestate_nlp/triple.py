from spacy.tokens import Token
from typing import Optional, Set, Union, Tuple
from ravestate_nlp.question_word import QuestionWord
from ravestate_nlp.triple_match_result import TripleMatchResult


class Triple:

    _subject: Union[Token, QuestionWord]
    _predicate: Token
    _object: Union[Token, QuestionWord]
    _yesno_question: bool

    def __init__(self, subject: Token = None, predicate: Token = None, object: Token = None):
        self.set_subject(subject)
        self.set_predicate(predicate)
        self.set_object(object)

    def set_subject(self, subject: Union[Token, QuestionWord]):
        self._subject = subject

    def set_predicate(self, predicate: Token):
        self._predicate = predicate

    def set_object(self, object: Union[Token, QuestionWord]):
        self._object = object

    def set_yesno_question(self, is_yesno: bool):
        self._yesno_question = is_yesno

    def get_subject(self) -> Union[Token, QuestionWord]:
        return self._subject

    def get_predicate(self) -> Token:
        return self._predicate

    def get_object(self) -> Union[Token, QuestionWord]:
        return self._object

    def get_yesno_question(self) -> bool:
        return self._yesno_question

    def has_subject(self) -> bool:
        return self._subject and len(self._subject.text.strip()) > 0

    def has_predicate(self) -> bool:
        return self._predicate and len(self._predicate.text.strip()) > 0

    def has_object(self) -> bool:
        return self._object and len(self._object.text.strip()) > 0

    def to_tuple(self) -> Tuple[str, str, str]:
        sub = ""
        pre = ""
        obj = ""
        if self._subject and not self._subject.is_space:
            sub = self._subject.text
        if self._predicate and not self._predicate.is_space:
            pre = self._predicate.lemma_
        if self._object and not self._object.is_space:
            obj = self._object.text
        return sub, pre, obj

    def match_either_lemma(self, pred: Optional[Set[str]] = None, subj: Optional[Set[str]] = None, obj: Optional[Set[str]] = None):
        """
        Checks if predicate, subject and/or object of the sentence or their children matches with the input set of word
        in lemma form. Returns TripleMatchResult object which consists of the match results with the corresponding
        matched words.
        """

        result = TripleMatchResult()

        def add_common_set(phrases: Optional[Set[str]], token, dep_relationship_whitelist=None, pos_relationship_whitelist=None):
            matches = ()
            if phrases:
                if token.lemma_ in phrases:
                    matches += (token.lemma_,)
                for child in token.children:
                    if child.lemma_ in phrases and \
                            (not dep_relationship_whitelist or child.dep_ in dep_relationship_whitelist) and \
                            (not pos_relationship_whitelist or child.pos_ in pos_relationship_whitelist):
                        matches += (child,)
            return matches

        from ravestate_nlp.extract_triples import PREDICATE_AUX_SET, VERB_AUX_SET

        result.preds += add_common_set(pred, self._predicate, dep_relationship_whitelist=PREDICATE_AUX_SET, pos_relationship_whitelist=VERB_AUX_SET)
        result.subs += add_common_set(subj, self._subject)
        result.objs += add_common_set(obj, self._object)

        return result

    def is_question(self, question_word: Optional[str] = None):
        if question_word:
            return self.match_either_lemma(obj={question_word}, subj={question_word})
        else:
            return self._yesno_question \
                   or isinstance(self._subject, QuestionWord) \
                   or isinstance(self._object, QuestionWord)

    def ensure_notnull(self, empty_token):
        # do not allow empty entries in triple
        if not self._subject:
            self._subject = empty_token
        if not self._predicate:
            self._predicate = empty_token
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
