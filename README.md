[![Release](https://img.shields.io/github/release/Roboy/ravestate.svg)](https://github.com/roboy/ravestate)
[![Build Status](https://travis-ci.org/Roboy/ravestate.svg?branch=master)](https://travis-ci.org/Roboy/ravestate)
[![codecov](https://codecov.io/gh/Roboy/ravestate/branch/master/graph/badge.svg)](https://codecov.io/gh/Roboy/ravestate)

## About

```
   ____                          __      __        _____       _____   
  / _  \____  __  ______  ______/ /_____/ /___    /_   _\     /_   _\  
 / /_/ / __ \/ / / / __ \/ ___\, / __ \, / __ \    0>  0>     <0  <0   
/ ,\ ,/ /_/ /\ \/ / /_/ /\__, / / /_/ / / /_/ /   \__⊽__/     \__⊽__/  
\/  \/\__/\/  \__/ ,___/\____/\/\__/\/\/ ,___/       ⋂  - Hey!   ⋂     
                 \____/                \____/             Olà! -       
```

Ravestate is a reactive library for real-time natural language dialog systems. It combines elements from event-based and reactive programming into an API, where application states are defined as functions that are run when a certain boolean set of criteria (signals) in the current application context is satisfied. It is the first reactive API to allow for boolean combinations of events. You may find a short introductory video [here](http://www.youtube.com/watch?v=6GMmY-xvA_Y "Introduction to Ravestate").

### Reactive Hello World

```python
import ravestate as rs

# We want to write some text output, so we
# need the raw:out context property from ravestate_rawio.
import ravestate_rawio as rawio

# Make sure that we use some i/o implementation,
# so we can actually see stuff that is written to rawio:out.
import ravestate_conio

# Ravestate applications should always be wrapped in a Module.
# This allows easier scoping, and enables separation of concerns
# beyond states.
with rs.Module(name="hi!", depends=(rawio.mod,)):

    # Create an application state which reacts to the `:startup` signal,
    # and writes a string to raw:out. Note: State functions are
    # always run asynchronously!
    @rs.state(cond=rs.sig_startup, write=rawio.prop_out)
    def hello_world(context):
        context[rawio.prop_out] = "Waddup waddup waddup!"

# Run context with console input/output and our 'hi!' module.
rs.Context("hi!").run()
```

### Raveboard

Ravestate has an [angular](https://angular.io)/[socket.io](https://socket.io)-based
interactive (beta) UI called __Raveboard__. It shows the events (spikes) that are
currently relevant, as well as potential state activations that are referencing these spikes.

When using `raveboard.UIContext` instead of `Context`, or `python3 -m raveboard` instead of
`python3 -m ravestate`, a real-time visualization of all spikes/activations, as well as a chat window,
will be hosted on a configurable port.

The following GIF shows raveboard together with `ravestate_visionio`:

[Raveboard](resources/docs/raveboard.gif)

## Installation

### Via PIP

The easiest way to install ravestate is through pip:

``
pip install ravestate
``

__Note:__ Ravestate requires Python 3.6 or higher. It is tested
on Ubuntu 16.04 and 18.04, as well as macOS > High Sierra.
It is currently not tested on Windows.

For reliability, we recommend using an environment virtualization tool,
like [virtualenv](https://virtualenv.pypa.io/en/latest/)
or [conda](https://conda.io/en/latest/).

### Via Docker/Docker-compose

Ravestate offers a docker image that bundles all dependencies that are required
for advanced cognitive chatbot design:

* `Neo4j`: The Neo4j graph DBMS is used by `ravestate_ontology` to provide long-term memory.
* `Redis`: The Redis in-memory DBMS is used to provide fast short-term memory, for example to store facial feature vectors.
* ``

There is a Dockerfile for ROS, ROS2 and Face Recognition support, which can be built with
```bash
docker build -t ravestate .
```
The image contains ROS, ROS2 and a ROS Bridge to connect ROS with ROS2.
Furthermore the roboy_communication message and service types are installed.

A container can then be created with docker-compose.yml. Choose `linux` or `macos` as fit for your platform:
```bash
docker-compose up -d rs-{linux|macos}
```

The container is now running and a connection into the container can be
established with:
```bash
docker exec -it rs bash
```

Inside the container, first source the ROS/ROS2 setups. Then 
ravestate can be run with `rclpy` (ROS2) and `rospy` (ROS) available.
```bash
# Source ROS2 setup if needed
. ~/ros2_ws/install/setup.bash

# Source ROS1 setup if needed
. ~/melodic_ws/idevel/setup.bash

# Start ravestate or raveboard to run your modules.
python3 -m ravestate [...]
```

**Note:** The ravestate docker container contains both `redis` and `neo4j`
databases, which are automatically started and mapped into folders at
`ravestate/db/{redis|neo4j}`.

### For developers and ROS users

#### Initial configuration and setup

Clone the repository and install dependencies:

```bash
cd ~

# Create a virtual python environment to not pollute the global setup
python3 -m virtualenv -p python3 python-ravestate

# Source the virtual environment
. python-ravestate/bin/activate

# Clone the repo
git clone git@github.com:roboy/ravestate && cd ravestate

# Install normal requirements
pip install -r requirements.txt

# To run tests & build docs, install pytest, mocking, fixtures, pydoc, ...
pip install -r requirements-dev.txt

# Link your local ravestate clone into your virtualenv
pip install -e .
```

Now, launch a Neo4j docker instance to serve [Scientio](https://github.com/roboy/scientio), so the dialog system has a memory:
```bash
docker run \
    --publish=7474:7474 --publish=7687:7687 \
    --volume=$HOME/neo4j/data:/data \
    --volume=$HOME/neo4j/logs:/logs \
    neo4j:latest
    
# Open the address localhost:7474 in a browser, and enter the
# credentials `neo4j`/`neo4j`. You will then be prompted to enter
# your own password. Remember this password.
```

In the `config` folder, create a file called `keys.yml`. It should have the following content:

```yaml
module: telegramio
config:
  telegram-token: <sexycactus>  # This is where your own telegram bot token
                                # will go later
---
module: ontology
config:
  neo4j_address: bolt://localhost:7687  # Your neo4j server uri here
  neo4j_username: neo4j                 # Your neo4j user here
  neo4j_pw: test                        # Your neo4j pw here
```

You may now conduct your first conversation with ravestate:
```bash
python3 -m ravestate -f config/generic.yml -f config/keys.yml
```

After the conversation, check the Neo4j interface under `localhost:7474`. It should now contain some nodes!

__Reminder: Whenever you use ravestate from the command line, source the virtual environment first!__

#### Running your Telegram bot

To test your telegram bot with a custom bot token in your `keys.yml`,
just run `telegram_test.yml` instead of `generic.yml`. This will load the `ravestate_telegramio` module.

#### Setting up PyCharm

1. Open your local ravestate clone as a project in pycharm.
2. Under `Project Preferences > Python interpreter`, set your virtual environment.
3. Mark the `modules` folder as sources root via the right-click context menu.
4. Create a run config via the "Edit configurations menu":<br>
   • Create a new Python configuration.<br>
   • Set `modules/ravestate/__main__.py` as the script to execute<br>
   • Set the working directory to the git clone directory.<br>
   • Set parameters to `-f config/generic.yml -f config/keys.yml`.<br> 
5. You should now be able to run the generic ravestate config from pycharm.

## Running Hello World

Ravestate applications are defined by a configuration,
which specifies the ravestate modules that should be loaded.

To run the basic hello world application, run ravestate
with a config file or command line arguments:

### Running with command line spec

You can easily run a combination of ravestate modules in a shared context,
by listing them as arguments to `python3 -m ravestate`:

```bash
python3 -m ravestate \
    ravestate_wildtalk \
    ravestate_conio \
    ravestate_hibye \
    ravestate_persqa
```
Run `python3 -m ravestate -h` to see more options!

### Running with config file(s) 

You may specify a series of config files to configure ravestate context,
when specifying everything through the command line becomes too laborious:

```yaml
# In file hello_world.yml
module: core
config:
  import:
    - ravestate_wildtalk
    - ravestate_conio
    - ravestate_hibye
    - ravestate_persqa
```
Then, run `ravestate` with this config file:

```bash
python3 -m ravestate -f hello_world.yml
```

## Module overview

Ravestate offers a landscape of fine-grained modules
for different aspects of dialog application tasks, which
may be seen in the following dependency diagram. Broadly,
the modules are categorized into Core (Blue), I/O (Yellow),
External (Red) and Skills (Green):

<img src="resources/docs/modules_sm.png">

#### Core Modules
  
  | Module name          | Description |
  |----------------------|-------------|
  | ravestate            | Core ravestate library.
  | ravestate_rawio      | Provides `raw_in`, `raw_out`, `pic_in` properties, which are served by the IO modules.
  | ravestate_ontology   | Connects to [scientio](https://github.com/roboy/scientio) to serve a built-in ontology.
  | ravestate_interloc   | Provides the `all_interlocutors` property, where present interlocutors are registered by the IO modules.
  | ravestate_idle       | Provides `bored` and `impatient` signals, as specified [here](https://github.com/Roboy/ravestate/issues/12).
  | ravestate_verbaliser | Utilities for easy management of conversational text, documented [here](modules/ravestate_verbaliser/README.md).
  | ravestate_nlp        | Spacy-based NLP properties and signals, documented [here](modules/ravestate_nlp/README.md).
  | ravestate_emotion    | Generates signals for, and recognizes specific emotions (`sig_shy`, `sig_surprise`, `sig_happy`, `sig_affectionate`).
  | ravestate_ros1       | Provides specific `Ros1PubProperty`, `Ros1SubProperty` and `Ros1CallProperty` context properties, which greatly simplify working with ROS1 in ravestate. Documentation [here](modules/ravestate_ros1/README.md).
  | ravestate_ros2       | Provides specific `Ros2PubProperty`, `Ros2SubProperty` and `Ros2CallProperty` context properties, which greatly simplify working with ROS2 in ravestate.


#### IO Modules

  | Module name          | Description |
  |----------------------|-------------|
  | ravestate_conio      | Simple command-line based IO for development purposes.
  | ravestate_telegramio | Single- or Multi-process Telegram server module, documented [here](modules/ravestate_telegramio/README.md). 
  | ravestate_roboyio    | [PyroBoy](https://github.com/roboy/pyroboy) -based STT/TTS with ROS2.
  | ravestate_visionio   | See dedicated docs [here](modules/ravestate_visionio/README.md). Enables face-recognition based dialog interactions.


#### Skill Modules

  | Module name          | Description |
  |----------------------|-------------|
  | ravestate_wildtalk   | See docs [here](modules/ravestate_wildtalk/README.md) - runs generative language models (GPT-2, ConvAi, ParlAi)!
  | ravestate_hibye      | Simply voices __Hi!__ (or the likes thereof) when an interlocutor is added, and __Bye__ when one is removed.
  | ravestate_persqa     | Conducts personalized smalltalk with interlocutors, interacts with Scientio to persist trivia.
  | ravestate_genqa      | [DrQA](https://github.com/roboy/drqa) -based general question answering module.
  | ravestate_roboyqa    | QA module which provides answers to questions about Roboy, such as __Who is your dad?__
  | ravestate_akinator __(*)__ | Enables dialog-based play of [Akinator!](modules/ravestate_akinator/README.md)
  | ravestate_sendpics __(*)__ | Uses face recognition to extract facial features and an assiciated Person with `pic_in` and ontology, which are then persisted in Redis and Scientio.
  | ravestate_fillers    | Recognize when the dialog context is taking a long time to produce an answer, and voice a filler like __"Uhm"__ or __"Let's see..."__.

**Note:** __(*)__ = deprecated.

## Running tests

If you have built the ravestate docker image as described above,
you may run the test suite as follows:

```bash
docker run -t -v $(pwd):/ravestate -w /ravestate ravestate ./run_tests.sh
```

## Building/maintaining the docs

If you have installed the dependencies from ``requirements-dev.txt``,
generate the docs by running this command at project root:

```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
git rm -rf docs
rm -rf _build docs
pydocmd build
mkdir -p docs/resources/docs && cp resources/docs/*.png docs/resources/docs && cp resources/docs/*.gif docs/resources/docs
git add docs/*
# For inspection: python3 -m http.server --directory docs
```

The structure and content of the docs are defined in the file ``pydocmd.yml``.
