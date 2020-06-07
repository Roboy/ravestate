#!/usr/bin/env bash

uwsgi \
    -H ~/py37 \
    --http :5000 \
    --wsgi-file uwsgi_mgr.py \
    --enable-threads \
    --callable app \
    --pyargv "uwsgi_mgr.yml" \
    --die-on-term
