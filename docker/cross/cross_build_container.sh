#!/bin/bash

cd ../resources

docker buildx use tj2-builder

for arch in amd64 arm64 ; do 
    docker buildx build \
    --platform linux/$arch \
    --output type=docker \
    -f ./Dockerfile \
    --tag tj2_ros-${arch}:latest .
done
