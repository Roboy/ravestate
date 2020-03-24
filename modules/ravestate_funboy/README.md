# Ravestate Funboy Module

```
    ______            __                    ____   ____   ______        ____    
   / ____/_  __ ___  / /   ____  __        /    \ /    \ /     //     //    \ /     /   
  / /__./ / / / __ \/ /_  / __ \/ / /\    /     //     //_____//     //     //_____/
 / .__./ /_/ / / / / ._ \/ /_/ / /_/ /   /     //_____//    \ /     //_____//     /
/ /    \__/\/\/ /_/ /_/ /\____/\__/ /   /\____//     //     / \____//     //     / 
\/                \____/     /\__/ /
                             \____/     
```

Funboy is an implementation of the DARVAH framework for the Ravestate Dialogue System.

## Running

Install the necessary dependencies via pip:
 
```bash

pip install -r requirements.txt

```

you also might need development dependencies:
 
```bash

pip install -r requirements-dev.txt

```

Make sure you properly installed and started:
* Funboyn4j - a Docker container with a Neo4j database instance 
* Rosemo - a Docker container with ROS-enabled emotion recognition modules for video and audio (you would need a web camera and a microphone)
* GPT-2-server - a simple http wrapper around the GPT-2 TLH model for humour generation
* Ravestate Wildtalk server - a server providing ConvAI wildtalk functionality 

Then, in the ravestate main directory, do:
```bash

pip install -e .

```

Afterwards, you can run Funboy by executing the following command:
```bash

python -m ravestate -f config/funboy.yml

```

The current Funboy conversation loop looks like this:
1. The greeting – in this part, you can exchange greetings with Roboy.
2. The conventional dialogue – in this part, Roboy asks or answers simple ques-
tions based on the ConvAI model inference.
3. The humour-enabled dialogue – in this part, Roboy starts to decide whether to tell jokes or use conventional responses. 
The part comes into effect after 60 seconds from the start.
4. The farewell - in this part, you can finish the conversation by saying
farewells to Roboy.

You can force Roboy to tell a joke by directly asking him to tell a joke (Funboy will react to token "joke").

