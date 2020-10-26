import os
from typing import Set
import re

from reggol import get_logger
logger = get_logger(__name__)

bad_words_path = os.path.join(os.path.dirname(__file__), "bad-words.txt")
stupid_fast_tokenize = re.compile(r"\b(\w+)\b")

with open(bad_words_path, 'r') as bad_words_file:
    bad_word_collection = {word.strip().lower() for word in bad_words_file.readlines()[1:]}

logger.info(f"Loaded {len(bad_word_collection)} forbidden words from {bad_words_path}.")

def is_bad_word(word: str):
    global bad_word_collection
    return word.strip().lower() in bad_word_collection

def contains_bad_word(utterance: str):
    return any(is_bad_word(word.group(0)) for word in stupid_fast_tokenize.finditer(utterance))
