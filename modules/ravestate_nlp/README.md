```
(C) Roboy 2019            
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

| Feature                           | Ravestate Properties/Signals  | Description                                                                               | Example Sentence: 'Revolutions need six fancy chickens!'|
| -------------                     | --------------------          |-------------------------------                                                            | ------------------------------|
| Tokenization                      | "nlp:tokens" (property)       | Segmenting text into words, punctuation marks etc.                                        | 'Revolutions', 'need', 'six', 'fancy', 'chickens', '!'|
| Part-of-Speech (POS) Tagging      | "nlp:postags" (property)      | Assigning word types to tokens                                                            | 'NOUN', 'VERB', 'NUM', 'ADJ', 'NOUN', 'PUNCT' |
| Detailed POS Tag                  | "nlp:tags" (property)         | Fine-grained part-of-speech                                                               | 'Revolutions' has the tag: 'NNS', which stand for: noun, plural <br> 'need' has the tag: 'VBP', which stands for: verb, non-3rd person singular present <br> [List](https://spacy.io/api/annotation#pos-tagging)  of POS tags|
| Lemmatization                     | "nlp:lemmas" (property)       | Assigning the base forms of words                                                         | 'Revolutions' has the lemma: 'revolution' <br>  'was' would have the lemma: 'be'|
| Named Entity Recognition (NER)    | "nlp:ner" (property)          | Labelling "real-world" objects                                                            | 'six' has the NER: 'CARDINAL', which are numerals that do not fall under another type <br>  [List](https://spacy.io/api/annotation#named-entities)  of NERs|
| Dependency Parsing                |                               | Assigning syntactic dependency labels, describing the relations between individual tokens | 'six' is a 'nummod' (numeric modifier) for 'chickens' <br> [List](https://spacy.io/api/annotation#dependency-parsing)  of dependencies <br> [displaCy](https://explosion.ai/demos/displacy) Dependency Visualizer|
| Triple Extraction                 | "nlp:triples" (property)      | A triple consists of subject, predicate, object of sentence                               | Triple: subject: 'Revolutions', predicate: 'need', object: 'chickens' |
| About Roboy                       | "nlp:roboy"  (property)       | Detecting whether sentence is about Roboy                                                 | 'you', 'roboy', 'robot', 'roboboy', ... |
| Yes-No                            | "nlp:yesno" (property)        | Detecting answers to yes-no questions                                                     | Checking for 'yes', 'no', 'i don't know', 'probably', 'probably not' and synonyms of these                           |
| Sentence Type: Question           | "nlp:is-question" (signal)    | Detecting whether sentence is a question                                                  |                                                |
| Play Game                         | "nlp:intent-play" (signal)    | Interlocutor wants to play a game                                                         | input: "I want to play", "I like games" or something similar    |

### Using the Features

#### React to Property Change
Each feature is stored in a ravestate property. 
A state which wants to access a property needs to read permission fot that property.

Example:

```python
@state(
    cond=s("nlp:yesno:changed"),  # state reacts to a change in the 'yesno' property
    read="nlp:yesno",  # state is allowed to read the 'yesno' property
    write="rawio:out")  # state is allowed to write to the output chanel 
def postive_chicken(ctx: ContextWrapper):
    if ctx["nlp:yesno"] == "yes":
        ctx["rawio:out"] = "You seem to be a positive chicken!"

```

#### React to Signal Emission
For 'Sentence Type: Question' a signal is emitted. 
A state that reacts to a signal can look something like this: 


```python
@state(
    cond=s("nlp:is-question"),  # state reacts to the is-question signal
    write="rawio:out")  # state is allowed to write to the output chanel 
def curious_chicken(ctx: ContextWrapper):
    ctx["rawio:out"] = "You seem to ask a lot of question, chicken!"

```

### Using the Triples for Sentence Analysis
The triple extraction is done in extract_triples.py by using the dependency tree of the sentence. 
A dependency tree shows the relation between the words of a sentence.
The finite verb (predicate) is the structural center of the sentence and therefor of the tree.
So starting with the predicate the algorithm searches through the dependency tree to find subject and object.

#### Analyzing a Sentence
Example: 'Chickens like revolutions' reaction state
Triple: subject: 'Chickens', predicate: 'like', object: 'revolutions'

```python
@state(
    cond=s("nlp:triples:changed"),  # state reacts to a change in the 'triples' property
    read="nlp:triples",  # state is allowed to read the 'triples' property
    write="rawio:out")  # state is allowed to write to the output chanel 
def postive_chicken(ctx: ContextWrapper):
    triple = ctx["nlp:triples"][0]  # gives you a Triple object
    if triple.match_either_lemma(subj={"chicken", "dinosaur"}):  
        # match_either_lemma() is a method in the Triple class 
        # returns true when subj, pred or obj have the desired value
        ctx["rawio:out"] = "You said something about a chicken, i like chickens!"
    # check predicate and object correspondingly

```

#### Happy language processing to all chickens out there!