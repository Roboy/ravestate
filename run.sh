#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
python3 run.py "$@"