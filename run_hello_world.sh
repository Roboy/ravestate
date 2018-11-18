#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
python3 run.py ravestate_conio hello_world -f config/keys.donotcommit.yml