```
                   _           _  _                  
                  | |         | |(_)                 
 _   _ _____  ____| |__  _____| | _  ___ _____  ____ 
| | | | ___ |/ ___)  _ \(____ | || |/___) ___ |/ ___)
 \ V /| ____| |   | |_) ) ___ | || |___ | ____| |    
  \_/ |_____)_|   |____/\_____|\_)_(___/|_____)_|    
                                                                                                                      
```

## Verbaliser
The Verbaliser produces Roboy's utterances. 
It diversifies the interactions with Roboy by randomizing the output given a specific intent.  

### Using the Verbaliser 

#### Question-Answer Lists
YAML files are used to define the actual utterances. 
In other words: they store everything Roboy can vocalise. 
To diversify his remarks the Verbaliser randomises similar outputs.

The class [QAPhrases](qa_phrases.py) retrieves the values from a YAML file and parses the containing phrases. 
Here is a template for such a YAML file:

```
type: qa                                    # remark types: question answering (qa), phrases
name: "INTENT"                              # intent or topic
Q:                                          # possible questions
- "Question phrasing 1"
- "Question phrasing 2"
- "Question phrasing 3"
A:                                          # answers 
  SUCCESS:
  - "Possible answer on success 1"
  - "Possible answer on success 2"
  FAILURE:
  - "Possible answer on failure"
FUP:                                        # follow up questions (for interlocutors that are already known to Roboy)
  Q:
  - "Possible follow up question"
  A:
  - "Possible follow up answer"
```

See more examples [here](../../resources/sentences).

#### Example for Answering the Question: What happened to the Dinosaurs?
* Creating the YAML file:

Fill in all the possible answers.  
```
type: qa
name: "DINO"
A:
  SUCCESS:
  - "I am sure it was a mind-boggingly huge meteorite!"
  - "They smoked too much ash!"
  - "A vulcano had flatulences."
  - "The chicken were stronger."
  FAILURE:
  - "I have no idea what you just said."
  - "Sorry, I am only interested in dinosaurs."
```
* Adding this file to the Verbaliser:

In this case the file is going to be located in a folder of important facts.
However, the single file can similarly be added by itself.
The folder is in the same path as the python file adding it.
```python
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "important_facts_folder"))
```

* Using the Verbaliser for fancy outputs:

This outputs an answer to the question.
To understand how to analyse the context of the question have a look at the [Natural Language Processing README](../ravestate_nlp/README.md)
```python
if input_had_something_to_do_with_dinos and was_a_question:
    ctx["rawio:out"] = verbaliser.get_random_successful_answer("DINO")
else:
    ctx["rawio:out"] = verbaliser.get_random_failure_answer("DINO")
```

* Possible conversation flow:
```
Interlocutor: "What happend to the Dinosaurs?"
Roboy: "The chicken were stronger."
```

#### Example for Extracting Phrase Lists
The Verbaliser can also be used to get all the imported phrases for a specific intent as a list. 

* Creating the phrases.yml:

```
type: phrases
name: "dino"
opts:
- "Dinos can not scratch their backs."
- "Once upon a time these mind-bogglingly huge creatures wandered the earth."
- "The longest Dinosaur was the Argentiosaurus."
---
type: phrases
name: "chicken"
opts:
- " Chickens are not completely flightless."
- " There are more chickens out there than programmers."
- " If I were a chicken for one day I would say: 'Puk Puk Pukaaak'. 

```

* Adding the file to the Verbaliser:

The YAML file is assumed to be located in the important_phrases folder. 
The folder is again in the same path as this python script: 

```python
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_file(join(dirname(realpath(__file__)), "important_phrases", "phrases.yml"))
```

* Using the Verbaliser to get a list of phrases:

Given a specific intent the Verbaliser can be used to return a list of phrases.

```python
import ravestate_verbaliser
ravestate_verbaliser.verbaliser.get_phrase_list('dino')
```
This will return a list of phrases for the given intent. 

#### The "verbaliser:intent" Property
The ["verbaliser:react_to_intent"](__init__.py) state produces a random phrase output for a given intent. 
All the possible intents are specified in YAML files that are located in the 
[ravestate_phrases_basic_en folder](../ravestate_phrases_basic_en). 

The state reads the "verbaliser:intent" property and outputs one random phrase in the list with that specific intent. 
It can therefor be triggered as follows:

Let's assume that phrases.yml is now located in avestate_phrases_basic_en.

```python
@state(cond=s("triggered_by_some_signal"), write="verbaliser:intent")
def say_some_nice_chicken_suff(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "chicken"
```