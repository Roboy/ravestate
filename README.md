
```
v0.1.0                         __      __
  __  ____  _   ______  ______/ /_____/ /___
 /  \/ __ \/ / / /__  \/ ___\, / __ \, /__  \
/ /\/ /_/ /\ \/ / /_/ /\__, / / /_/ / / /_/ /
\/  \__/\/  \__/ ,___/\____/\/\__/\/\/ ,___/
               \____/                \____/
```

## About

Ravestate is a reactive library for real-time natural language dialog systems.

## Requirements:

- Python >= 3.6
## Installation

To install dependencies use:

```bash
pip install -r requirements.txt

# To run tests, install pytest, mocking, fixtures...
pip install -r requirements-dev.txt
```

### Mac

#### PyAudio

In order to install PyAudio with pip, you need to install portaudio first using:

``
brew install portaudio
``
## Running Hello World

Ravestate applications are defined by a configuration,
that specifies the ravestate modules that should be loaded.

To run the basic hello world application, run ravestate with the proper config file:

```bash
./run.sh -f config/hello_world.yml
```

## Running tests

If you have installed the dependencies from ``requirements-dev.txt`` you
may run the ravestate test suite as follows:

```bash
./run_tests.sh
```
