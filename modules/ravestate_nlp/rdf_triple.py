from spacy.tokens import Token


class Triple:

    _subject = None
    _predicate = None
    _adjective = None
    _object = None

    def __init__(self, subject: Token = None, predicate: Token = None, _adjective: Token=None, object: Token = None):
        self.set_subject(subject)
        self.set_predicate(predicate)
        self.set_object(object)

    def set_subject(self, subject: Token):
        self._subject = subject

    def set_predicate(self, predicate: Token):
        self._predicate = predicate
    
    def set_predicate_subplement(self, adjective: Token):
        self._adjective = adjective

    def set_object(self, object: Token):
        self._object = object

    def get_subject(self) -> Token:
        return self._subject

    def get_predicate(self) -> Token:
        return self._predicate

    def get_adjective(self) -> Token:
        return self._adjective

    def get_object(self) -> Token:
        return self._object

    def to_tuple(self) -> tuple:
        return self._subject.text, self._predicate.text, self._adjective.text, self._object.text

    def __eq__(self, other) -> bool:
        if isinstance(other, self.__class__):
            if self._subject.text != other.get_subject().text:
                return False
            if self._predicate.text != other.get_predicate().text:
                return False
            if self._adjective.text != other.get_adjective().text:
                return False
            if self._object.text != other.get_object().text:
                return False
            return True
        if isinstance(other, tuple) and isinstance(other[0], str) and len(other) == 3:
            if self._subject.text.lower() != other[0].lower():
                return False
            if self._predicate.lemma_.lower() != other[1].lower():
                return False
            if self._adjective.lemma_.lower() != other[1].lower():
                return False
            if self._object.text.lower() != other[2].lower():
                return False
            return True
        return False

    def __str__(self):
        sub = ''
        pred = ''
        adj = ''
        obj = ''
        space = ''
        if self._subject:
            sub = self._subject.text
        if self._predicate:
            pred = self._predicate.lemma_
        if self._adjective:
            adj = self._adjective.lemma_
            space = ' '
        if self._object:
            obj = self._object.text
        return f'{sub}:{pred}{space}{adj}:{obj}'.lower()

    def __repr__(self):
        sub = ''
        pred = ''
        adj = ''
        obj = ''
        space = ''
        if self._subject:
            sub = self._subject.text
        if self._predicate:
            pred = self._predicate.lemma_
        if self._adjective:
            adj = self._adjective.lemma_
            space = ' '
        if self._object:
            obj = self._object.text
        return f'<Triple object {sub}:{pred}{space}{adj}:{obj}>'
