```
(C) Roboy 2019            
                   _           _  _                  
                  | |         | |(_)                 
 _   _ _____  ____| |__  _____| | _  ___ _____  ____ 
| | | | ___ |/ ___)  _ \(____ | || |/___) ___ |/ ___)
 \ V /| ____| |   | |_) ) ___ | || |___ | ____| |    
  \_/ |_____)_|   |____/\_____|\_)_(___/|_____)_|    
                                                                                                                      
```

## Verbaliser
The Verbaliser produces Roboy's utterances. It diversifies the interactions with Roboy.
The Verbaliser gives an random output given an certain intent. 
intent -> specific theme of what you want roboy to answer

### Using the Verbaliser 

#### YAML File with Desired Utterances
YAML files are used to define the actual utterances. 
In other words: they store everything Roboy can vocalise. 
To diversify his remarks the Verbaliser randomises similar outputs.

The class QAPhrases retrieves the values from a YAML file and parses the containing phrases. 
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

See more examples in resources / sentences

#### Example for Answering the Question: What happened to the Dinosaurs?
* Create the YAML file:
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
  - "I have no idea what you just saig."
  - "Sorry, I am only interested in dinosaurs."
```
* Add this file to the Verbaliser
In this case the file is going to be located in a folder of important facts.
However, the single file can similarly be added by itself.
The folder is in the same path as the python file adding it.
```python
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "important_facts_folder"))
```

* Use the Verbaliser for fancy outputs
This outputs an answer to the question.
To understand how to analyse the question have a look at the nlp readme. (LINK)
```python
if input_had_something_to_do_with_dinos and was_a_question:
    ctx["rawio:out"] = verbaliser.get_random_successful_answer("DINO")
else:
    ctx["rawio:out"] = verbaliser.get_random_failure_answer("DINO")
```

* Possible conversation flow
```
Interlocutor: "What happend to the Dinosaurs?"
Roboy: "The chicken were stronger."
```