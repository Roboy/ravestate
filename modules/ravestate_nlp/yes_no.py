NEGATION_SET = {"neg"}


def yes_no(doc):
    """
    checks input for "yes", "no", "i don't know", "probably" and "probably not"
    TODO: This should also map other inputs like ("(for|not) sure", "definitely, "dont think so"...)
    """
    nlp_tokens = tuple(str(token.text.lower()) for token in doc)
    for token in nlp_tokens:
        if token == "yes" or token == "y":
            return "yes"
        elif token == "no" or token == "n":
            return "no"
    for token in doc:
        if token.dep_ in NEGATION_SET:
            if "know" in nlp_tokens:
                return "idk"
            elif "probably" in nlp_tokens:
                return "pn"
        elif "probably" in nlp_tokens:
            return "p"
    return "0"

