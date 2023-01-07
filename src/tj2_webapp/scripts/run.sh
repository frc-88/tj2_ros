#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

cd ${BASE_DIR}/../src
gunicorn --worker-class gevent --workers 1 --bind 0.0.0.0:5802 tj2_webapp_node:app --max-requests 10000 --timeout 5 --keep-alive 5 --log-level info
