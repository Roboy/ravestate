```
       _        
      | |       
 ____ | | ____  
|  _ \| ||  _ \ 
| | | | || |_| |
|_| |_|\_)  __/ 
         |_|                                                                 
```

## NLP

The natural language processing (NLP) module enables Roboy to process and understand the human language. 
That way he can interpret the meaning of the detected sentences.  
We use a free, open-source NLP library for advanced NLP in Python: [spaCy](https://spacy.io/) 


### Extracted Features

| Feature                           | Ravestate Properties/Signals  | Description                                                 | Example Sentence: 'Revolutions need six fancy chickens!'|
| -------------                     | --------------------          |-------------------------------                              | ------------------------------|
| Tokenization                      | `nlp.prop_tokens`             | Segmenting text into words, punctuation marks etc.          | 'Revolutions', 'need', 'six', 'fancy', 'chickens', '!'|
| Part-of-Speech (POS) Tagging      | `nlp.prop_postags`            | Assigning word types to tokens                              | 'NOUN', 'VERB', 'NUM', 'ADJ', 'NOUN', 'PUNCT' |
| Detailed POS Tag                  | `nlp.prop_tags`               | Fine-grained part-of-speech                                 | 'Revolutions' has the tag: 'NNS', which stand for: noun, plural <br> 'need' has the tag: 'VBP', which stands for: verb, non-3rd person singular present <br> [List](https://spacy.io/api/annotation#pos-tagging) of POS tags|
| Lemmatization                     | `nlp.prop_lemmas`             | Assigning the base forms of words                           | 'Revolutions' has the lemma: 'revolution' <br>  'was' would have the lemma: 'be'|
| Named Entity Recognition (NER)    | `nlp.prop_ner`                | Labelling "real-world" objects ("NE"=Named Entity)          | 'six' has the NE: 'CARDINAL', which are numerals that do not fall under another type <br> [List](https://spacy.io/api/annotation#named-entities) of NEs|
| Triple Extraction                 | `nlp.prop_triples`            | A triple consists of subject, predicate, object of sentence | Triple: subject: 'Revolutions', predicate: 'need', object: 'chickens' |
| About Roboy                       | `nlp.prop_roboy`              | Detecting whether sentence is about Roboy                   | 'you', 'roboy', 'robot', 'roboboy', ... |
| Yes-No                            | `nlp.prop_yesno`              | Detecting answers to yes-no questions                       | Checking for 'yes', 'no', 'i don't know', 'probably', 'probably not' and synonyms of these                           |
| Sentence Type: Question           | `nlp.sig_is_question`         | Emitted if the input sentence is a question                 |                                                |
| Play Game                         | `nlp.sig_intent_play`         | Emitted when the interlocutor wants to play a game          | input: "I want to play", "I like games" or something similar    |

Additional features can be added and published to the system. All existing features can be found [here](__init__.py).

### Using the Features

#### React to Property Change
Each feature is stored in a ravestate property. 
A state which wants to access a property needs read permissions for that property.

Example: State that reads the "yesno" property

```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

@rs.state(
    cond=nlp.prop_yesno.changed(),  # state reacts to a change in the 'yesno' property
    read=nlp.prop_yesno,  # state is allowed to read the 'yesno' property
    write=rawio.prop_out)  # state is allowed to write to the output property
def postive_chicken(ctx: ContextWrapper):
    if ctx[nlp.prop_yesno].yes():
        ctx[rawio.prop_out] = "You seem to be a positive chicken!"

```

#### React to Signals
For 'Sentence Type: Question' a signal is emitted.

Example: State that reacts to the "is-question" signal 


```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

@rs.state(
    cond=nlp.sig_is_question,  # state reacts to the is-question signal
    write=rawio.prop_out)      # state is allowed to write to the output property 
def curious_chicken(ctx: ContextWrapper):
    ctx[nlp.prop_out] = "You seem to ask a lot of questions, chicken!"

```

### Using the Triples for Sentence Analysis
The triple extraction is done in [extract_triples.py](extract_triples.py) by using the dependency tree of the sentence. 
A dependency tree shows the relation between the words of a sentence.
The finite verb (predicate) is the structural center of the sentence and therefor of the tree.
So starting with the predicate the algorithm searches through the dependency tree to find subject and object.

#### Analyzing a Sentence
Example: 'Chickens like revolutions' reaction state
Triple: subject: 'Chickens', predicate: 'like', object: 'revolutions'

```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

@rs.state(
    cond=nlp.prop_triples.changed(),  # state reacts to a change in the 'triples' property
    read=nlp.prop_triples,  # state is allowed to read the 'triples' property
    write=rawio.prop_out)  # state is allowed to write to the output chanel 
def postive_chicken(ctx: ContextWrapper):
    triple = ctx[nlp.prop_triples][0]  # gives you the first Triple object
    # check predicate and object correspondingly
    # match_either_lemma() is a method in the Triple class 
    if triple.match_either_lemma(subj={"chicken", "dinosaur"}):  
        # returns true when subj, pred or obj have the desired value
        ctx[rawio.prop_out] = "You said something about a chicken, i like chickens!"
```

### Using User-defined Signals and Implementing Q/A-States
A user can define signals as needed and implement states reacting on them. 
If both a signal is emitted and something is written to `rawio` in one state, the state needs to be defined with `emit_detached=True`.

#### React to User-defined States and Implement Q/A
Example: In one state a question is asks and then a user-defined signal is emitted that shows that the question has been stated. The other states reacts on the answer to this question.
```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio
    
chicken_signal = rs.Signal("chicken signal")
    
@rs.state(
    cond=nlp.prop_tokens.changed(),
    write=rawio.prop_out,
    signal=chicken_signal,
    emit_detached=True
)
def ask_chicken_question(ctx: rs.ContextWrapper):
    ctx[rawio.prop_out] = "Wanna know something awesome about chickens?"
    return rs.Emit()
    
@rs.state(
    cond=chicken_signal & nlp.prop_yesno.changed(),
    read=nlp.prop_yesno,
    write=rawio.prop_out
)
def answer_chicken_question(ctx: rs.ContextWrapper):
    if ctx[nlp.prop_yesno] == "yes":
        ctx[rawio.prop_out] = "Well, chicken's actually aren't that awesome."
    elif ctx[nlp.prop_yesno] == "no":
        ctx[rawio.prop_out] = "You're missing out on awesome chicken stories!"
```


#### Happy language processing to all chickens out there!

### Hands-on

[How to use nlp features for selling ice cream.](ICE_CREAM_SELLING_NLP_USA_CASES.md)
