from ravestate_nlp.question_word import QuestionWord
from ravestate_nlp.triple import Triple
from spacy.tokens import Token

SUBJECT_SET = {"nsubj"}
OBJECT_SET = {"dobj", "attr", "advmod", "pobj"}
PREDICATE_SET = {"ROOT", "conj"}
PREDICATE_AUX_SET = {"acomp", "aux", "xcomp"}
VERB_AUX_SET = {"VERB", "ADJ"}
RECURSION_BLACKLIST = {"cc", "conj"}


def extract_triples(doc):
    """
    triple: subject, predicate, object
    finding the triple of a sentence:
    this is done by using the dependency tree of the sentence
    a dependency tree shows the relation between the words of a sentence
    the finite verb (predicate) is the structural center of the sentence and therefor of the tree
    so starting with the predicate the algorithm searches through the dependency tree to find subject and object
    -> triple_search()

    examples:
    - How are you?
     tree depth = 1     predicate: "are"
                            -> relation to "how": adverbial modifier (advmod) => object of this sentence
                            -> relation to "you": nominal subject (nsubj) => subject of this sentence

    - How old are you?
    tree depth = 2      predicate: "are"
                            -> relation to "old": adjectival complement (acomp) => auxiliary predicate
                                -> relation to "how":  adverbial modifier (advmod) => object of this sentence
                            -> relation to "you": nominal subject (nsubj) => subject of this sentence

    - Who is your father?
    tree depth = 2      predicate: "is"
                            -> relation to "who": nominal subject (nsubj) => subject of this sentence
                            -> relation to "father": attribute (attr) => object of this sentence
                                    ->relation to "your": possession modifier (poss)
    """
    triples = []
    for token in doc:
        if token.dep_ in PREDICATE_SET:
            triple = triple_search(Triple(predicate=token), token)
            triple.ensure_notnull(doc._.empty_token)
            triples.append(triple)
    return triples


def triple_search(triple: Triple, token: Token):
    """
    Recursive search through the dependency tree
    looks for triple values in each of the children and calls itself with the children nodes
    """
    question_word = None
    for word in token.children:
        if word.text.lower() in QuestionWord.question_words:
            question_word = QuestionWord(word)
            word = QuestionWord(word)
            if not triple.get_object():
                triple.set_object(question_word)
        elif word.dep_ in OBJECT_SET:
            triple.set_object(word)
        if word.dep_ in SUBJECT_SET:
            triple.set_subject(word)
        if word.dep_ in PREDICATE_AUX_SET and word.pos_ in VERB_AUX_SET:
            triple.set_predicate_aux(word)
        if isinstance(word, Token) and word.dep_ not in RECURSION_BLACKLIST:
            triple = triple_search(triple, word)
    if not triple.get_subject() and question_word:
        triple.set_subject(question_word)
    return triple
