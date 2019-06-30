## Sentence Processing and Feature Extraction Techniques for Ice Cream Selling

This document discusses the possible sentence processing and feature extraction techniques 
applied with NLP module of ravestate library for ice cream selling.

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
https://spacy.io/api/annotation#named-entities for further types and for Spacy's interactive NER detector https://explosion.ai/demos/displacy-ent

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
Triple Extraction provides object and subject applied with the verb. 
This allows us to have a wider perspective over the request because
we can specifically understand which flavor is additionally or less requested.
Consider the following scenarios:

- I need more vanilla ice cream
- Drop the chocolate please.

This feature can be used to understand the requested flavors or payment options. 
Roboy's flavor and payment options are predefined so any match between our database and incoming token
can help us to identify the requested options. Such as:

- *"vanilla"*
- *"chocolate"*
- *"lemon"*
- *"strawberry"*

```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

prop_flavor = rs.Property(name="flavor", allow_read=True, allow_write=True, always_signal_changed=True)

@rs.state(
    cond=nlp.prop_triples.changed(),  # state reacts to a change in the 'prop_triples' property
    read=nlp.prop_triples,  # state is allowed to read the 'prop_triples' property
    write=prop_flavor, # state is allowed to write to the 'prop_flavor' property
    signal=prop_flavor.changed_signal)  # state signals 'prop_flavor' change signal 
def flavor_recognition_state(ctx: ContextWrapper):
    triple_match_result = ctx[nlp.prop_triples][0].match_either_lemma(obj={"vanilla", "strawberry", "lemon", "chocolate"}):
    objects = triple_match_result.objs
    if objects:
        ctx[prop_flavor] = objects
        return rs.Emit()
```

or ...

- *"credit card"*
- *"cash"*
- *"bitcoin"*

```python
import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

prop_payment = rs.Property(name="payment", allow_read=True, allow_write=True, always_signal_changed=True)

@rs.state(
    cond=nlp.prop_triples.changed(),  # state reacts to a change in the 'prop_triples' property
    read=nlp.prop_triples,  # state is allowed to read the 'prop_triples' property
    write=prop_payment, # state is allowed to write to the 'prop_payment' property
    signal=prop_payment.changed_signal)  # state signals 'prop_payment' change signal 
def payment_recognition_state(ctx: ContextWrapper):
    triple_match_result = ctx[nlp.prop_triples][0].match_either_lemma(obj={"bitcoin", "cash", "card"}):
    objects = triple_match_result.objs
    if objects and len(objects) == 1:
        ctx[prop_payment] = objects[0] 
        return rs.Emit()
```

Alternatively by using triples, Roboy can understand whether the sentence is a request or not by only parsing the verb.
For example if the customer uses one of the following phrases, it is easy to detect that customer has a request:

- *"would like"*
- *"can have"*
- *"can get"*
- *"will have"*
- *"choose"*
- *"wish"*
- *"desire"*
- *"prefer"*
