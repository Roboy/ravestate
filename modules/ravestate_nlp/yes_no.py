from enum import Enum

NEGATION_TUPLE = ["neg", "not"]
YES_SYNONYMS = {"yes", "y", "yeah", "sure", "definitely", "certainly"}
NO_SYNONYMS = {"no", "n", "nope", "negative", "never", "nay"}
PROBABLY_SYNONYMS = {"probably", "likely", "possibly", "perhaps", "maybe"}
DO_NOT_KNOW_SET = {"know", "understand", "clue"}


class YesNo(Enum):
    YES = 2
    NO = -2
    PROBABLY = 1
    PROBABLY_NOT = -1
    DONT_KNOW = 0
    DEFAULT = DONT_KNOW


class YesNoWrapper:
    answer: YesNo

    def __init__(self, input_text):
        nlp_tokens = tuple(str(token.text.lower()) for token in input_text)
<<<<<<< HEAD
        if not set(nlp_tokens).isdisjoint(NEGATION_TUPLE):
            if not set(nlp_tokens).isdisjoint(PROBABLY_SYNONYMS):
                self.answer = YesNo.PROBABLY_NOT
                return
            elif not set(nlp_tokens).isdisjoint(DO_NOT_KNOW_SET):
                self.answer = YesNo.DONT_KNOW
                return
            elif not set(nlp_tokens).isdisjoint(YES_SYNONYMS):
                self.answer = YesNo.NO
                return
        elif not set(nlp_tokens).isdisjoint(YES_SYNONYMS):
            if not set(nlp_tokens).isdisjoint(PROBABLY_SYNONYMS):
                self.answer = YesNo.PROBABLY
                return
            elif not set(nlp_tokens).isdisjoint(NO_SYNONYMS):
                if self._is_confusing(nlp_tokens):
                    self.answer = None
=======
        nlp_token_dep = tuple(str(token.dep_) for token in input_text)
        for token in nlp_tokens:
            if token in PROBABLY_SYNONYMS:
                if NEGATION_TUPLE[0] in nlp_token_dep or NEGATION_TUPLE[1] in nlp_tokens:
                    self.answer = YesNo.PROBABLY_NOT
                    return
                else:
                    self.answer = YesNo.PROBABLY
                    return
            elif token in YES_SYNONYMS:
                if NEGATION_TUPLE[0] in nlp_token_dep or NEGATION_TUPLE[1] in nlp_tokens:
                    self.answer = YesNo.NO
                    return
                else:
                    self.answer = YesNo.YES
                    return
            elif token in NO_SYNONYMS:
                self.answer = YesNo.NO
                return
            elif token in DO_NOT_KNOW_SET:
                if NEGATION_TUPLE[0] in nlp_token_dep or NEGATION_TUPLE[1] in nlp_tokens:
                    self.answer = YesNo.DONT_KNOW
>>>>>>> 6464ba6593850af750914eece5c109b6a3ce1cb1
                    return
                self.answer = YesNo.YES
                return
            else:
                self.answer = YesNo.YES
                return
        elif not set(nlp_tokens).isdisjoint(NO_SYNONYMS):
            if not set(nlp_tokens).isdisjoint(DO_NOT_KNOW_SET):
                self.answer = YesNo.DONT_KNOW
                return
            self.answer = YesNo.NO
            return
        elif not set(nlp_tokens).isdisjoint(PROBABLY_SYNONYMS):
            self.answer = YesNo.PROBABLY
            return
        self.answer = None

    @staticmethod
    def _is_confusing(tokens):
        yes_index = tokens.index(set(tokens).intersection(YES_SYNONYMS).pop())
        no_index = tokens.index(set(tokens).intersection(NO_SYNONYMS).pop())
        if abs(yes_index - no_index) < 2:
            return True
        return False

    def yes(self):
        if isinstance(self.answer, YesNo):
            return self.answer.value if self.answer.value > 0 else False
        return None

    def no(self):
        if isinstance(self.answer, YesNo):
            return -self.answer.value if self.answer.value < 0 else False
        return None

    def unk(self):
        if isinstance(self.answer, YesNo):
            return self.answer == YesNo.DONT_KNOW
        return None

    def __str__(self):
        if isinstance(self.answer, YesNo):
            return f"{self.answer.name}:{self.answer.value}"
        return str(None)

    def __repr__(self):
        return f"<YesNoWrapper object {str(self)}>"


def yes_no(doc):
    """
    checks input for "yes", "no", "i don't know", "probably" and "probably not" and synonyms of these
    """
    return YesNoWrapper(doc)
