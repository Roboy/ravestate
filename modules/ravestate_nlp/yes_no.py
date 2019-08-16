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
        if len(nlp_tokens) == 1:
            if nlp_tokens[0] in YES_SYNONYMS:
                self.answer = YesNo.YES
                return
            elif nlp_tokens[0] in NO_SYNONYMS:
                self.answer = YesNo.NO
                return
            elif nlp_tokens[0] in PROBABLY_SYNONYMS:
                self.answer = YesNo.PROBABLY
                return

        nlp_token_dep = tuple(str(token.dep_) for token in input_text)
        if NEGATION_TUPLE[0] in nlp_token_dep or NEGATION_TUPLE[1] in nlp_tokens:
            for token in nlp_tokens:
                if token in PROBABLY_SYNONYMS:
                    self.answer = YesNo.PROBABLY_NOT
                    return
                elif token in YES_SYNONYMS:
                    self.answer = YesNo.NO
                    return
                elif token in DO_NOT_KNOW_SET:
                    self.answer = YesNo.DONT_KNOW
                    return
        self.answer = None

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
