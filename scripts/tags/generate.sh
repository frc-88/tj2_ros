#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

${BASE_DIR}/generate_tag.py -p 20 -l 165.1 -t pdf tag36h11 1 2 3 4
