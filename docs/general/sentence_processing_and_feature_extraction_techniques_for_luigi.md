## Sentence Processing and Feature Extraction Techniques for Ice Cream Selling

This document discusses the possible sentence processing and feature extraction techniques 
applied with NLP module of ravestate library for ice cream selling.

#### NLP - Extracted Features

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

### Using the Features

#### Detailed POS Tagging

This feature can be used to understand the requested flavors or payment options. 
Roboy's flavor and payment options are predefined so any match between our database and incoming token
can help us to identify the requested options. Such as:

- vanilla
- chocolate
- lemon
- strawberry

or 

- credit card
- cash
- bitcoin

Alternatively by using POS, Roboy can understand whether the sentence is a request or not by only parsing the verb.
For example if the customer uses one of the following phrases, it is easy to detect that customer has a request:

- would like
- can have
- can get
- will have
- choose
- wish
- desire
- prefer
- could do with
- have an urge for

#### Yes-No Property

This feature can be used to understand a given question's positive/negative answer to proceed such as:

- Do you want the usual?
- Do you want ice cream?

Example: State that reads the "yesno" property

```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

@rs.state(
    cond=nlp.prop_yesno.changed(),  # state reacts to a change in the 'yesno' property
    read=nlp.prop_yesno,  # state is allowed to read the 'yesno' property
    write=rawio.prop_out)  # state is allowed to write to the output property
def customer_wants_ice_cream(ctx: ContextWrapper):
    if ctx[nlp.prop_yesno] == "yes":
        ctx[rawio.prop_out] = "Your ice cream is coming right up!"

```

#### Lemmatization

After tokenizing the sentences, roboy needs to match the strings in order to understand the context.
Lemmatization allows to get the base form of words which is the required form for matching.


#### Named Entity Recognition (NER)

NER can be used in order to understand the quantity of ice cream, payment or any matter that requires specifics.
The following figure includes the most possible types of use, please check 
https://spacy.io/api/annotation#named-entities for further types.

| Type                              | DESCRIPTION
| -------------                     | --------------------          
| DATE                              | Absolute or relative dates or periods.
| TIME                              | Times smaller than a day.
| PERCENT                           | Percentage, including ”%“.                
| MONEY                             | Monetary values, including unit.    
| QUANTITY                          | Measurements, as of weight or distance.            
| ORDINAL                           | “first”, “second”, etc.           
| CARDINAL                          | Numerals that do not fall under another type.             


### Triple Extraction
Triple Extraction can be used to understand any customer request. In addition to Detailed POS,
Triple Extraction provides object and subject applied with the verb. 
This allows us to have a wider perspective over the request because 
we can specifically understand which flavor is additionally requested or not.
Consider the following scenarios:

- I need more vanilla ice cream
- Drop the chocolate please.
