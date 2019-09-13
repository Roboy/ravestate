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
        nlp_token_set = set(nlp_tokens)

        has_negation = not nlp_token_set.isdisjoint(NEGATION_TUPLE)
        has_probably = not nlp_token_set.isdisjoint(PROBABLY_SYNONYMS)
        has_dontknow = not nlp_token_set.isdisjoint(DO_NOT_KNOW_SET)
        has_yes = not nlp_token_set.isdisjoint(YES_SYNONYMS)
        has_no = not nlp_token_set.isdisjoint(NO_SYNONYMS)

        if has_negation:
            if has_probably:
                self.answer = YesNo.PROBABLY_NOT
                return
            elif has_dontknow:
                self.answer = YesNo.DONT_KNOW
                return
            elif has_yes:
                self.answer = YesNo.NO
                return
        if has_yes:
            if has_probably:
                self.answer = YesNo.PROBABLY
                return
            elif has_no:
                if self._is_confusing(nlp_tokens):
                    self.answer = None
                    return
                self.answer = YesNo.YES
                return
            else:
                self.answer = YesNo.YES
                return
        if has_no:
            if has_dontknow:
                self.answer = YesNo.DONT_KNOW
                return
            self.answer = YesNo.NO
            return
        if has_probably:
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
