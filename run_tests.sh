#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
pytest --cov=modules test -p no:warnings
