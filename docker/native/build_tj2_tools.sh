#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

cd ${BASE_DIR}/../resources

docker-compose -f docker-compose.build_tj2_tools.yml up -d
docker logs -f build_tj2_tools
