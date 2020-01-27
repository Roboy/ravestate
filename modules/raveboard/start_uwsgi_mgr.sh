#!/usr/bin/env bash

uwsgi \
    -H ~/py37 \
    --http :5000 \
    --wsgi-file uwsgi_mgr.py \
    --processes 1 \
    --enable-threads \
    --callable app \
    --pyargv "uwsgi_mgr.yml"
