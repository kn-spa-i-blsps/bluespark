#!/bin/bash

# let docker display windows
xhost +local:root 

docker run -it --rm \
  --name  bluespark-aut \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  -w /workspace \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \
  bluespark-ros2