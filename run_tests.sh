#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$(pwd)/modules
pytest --ignore ros2
