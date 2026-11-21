#!/bin/bash

# Check if Docker image exists, build if not
if ! docker image inspect revo2_description_ros2 >/dev/null 2>&1; then
    echo "Building revo2_description_ros2 Docker image..."
    docker build -t revo2_description_ros2 --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)  .docker 
    echo "Docker image built successfully."
else
    echo "Using existing revo2_description_ros2 Docker image."
fi

docker run -it -u $(id -u) \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
    -v $(pwd):/workspaces/src/revo2_description \
    -w /workspaces/src/revo2_description \
    revo2_description_ros2 \
    .docker/visualize_revo2.entrypoint.sh  $*
