NEGATION_TUPLE = ["neg", "not"]
YES_SYNONYMS = {"yes", "y", "yeah", "sure", "definitely", "	certainly"}
NO_SYNONYMS = {"no", "n", "nope", "negative", "never", "nay"}
PROBABLY_SYNONYMS = {"probably", "likely", "possibly", "perhaps", "maybe"}
DON_NOT_KNOW_SET = {"know", "understand", "idea"}


def yes_no(doc):
    """
    checks input for "yes", "no", "i don't know", "probably" and "probably not" and synonyms of these
    """
    nlp_tokens = tuple(str(token.text.lower()) for token in doc)
    if len(nlp_tokens) == 1:
        if nlp_tokens[0] in YES_SYNONYMS:
            return "yes"
        elif nlp_tokens[0] in NO_SYNONYMS:
            return "no"
        elif nlp_tokens[0] in PROBABLY_SYNONYMS:
            return "p"

    nlp_token_dep = tuple(str(token.dep_) for token in doc)
    if NEGATION_TUPLE[0] in nlp_token_dep or NEGATION_TUPLE[1] in nlp_tokens:
        for token in nlp_tokens:
            if token in PROBABLY_SYNONYMS:
                return "pn"
            elif token in YES_SYNONYMS:
                return "no"
            elif token in DON_NOT_KNOW_SET:
                return "idk"
    return "_"
