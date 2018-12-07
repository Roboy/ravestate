#!/usr/bin/env bash

export setupfile=${1}

if [[ ! $setupfile ]]; then
    export setupfile=setup.py
fi

rm -rf dist
python3 $setupfile sdist bdist_wheel
twine upload dist/*