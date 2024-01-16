#!/usr/bin/env bash

##### Credits to: https://github.com/osrf/icra2023_ros2_gz_tutorial/blob/main/docker/run.bash

#
# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [--rm] [--nvidia]"
    exit 1
fi

# Default to no NVIDIA
DOCKER_OPTS=""

# Parse and remove args
PARAMS=""
REMOVE_CONTAINER=""
while (( "$#" )); do
  case "$1" in
    --rm)
      REMOVE_CONTAINER="--rm"
      shift
      ;;
    --nvidia)
      # Note that on all non-Debian/non-Ubuntu platforms, dpkg won't exist so they'll always choose
      # --runtime=nvidia.  If you are running one of those platforms and --runtime=nvidia doesn't work
      # for you, change the else statement.
      if [[ -x "$(command -v dpkg)" ]] && dpkg --compare-versions "$(docker version --format '{{.Server.Version}}')" gt "19.3"; then
        DOCKER_OPTS="--gpus=all"
      else
        DOCKER_OPTS="--runtime=nvidia"
      fi
      shift
      ;;
    -*|--*=) # unsupported flags
      echo "Error: Unsupported flag $1" >&2
      exit 1
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done
# set positional arguments in their proper place
eval set -- "$PARAMS"

IMG="$1"


# Make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        touch $XAUTH
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


# Get the absolute path of the directory this script is in
CURRENT_DIR="$(pwd)"

# Relax X server permissions so that local X connections work; this is necessary
# when running under XWayland
xhost + local:

# --ipc=host and --network=host are needed for no-NVIDIA Dockerfile to work
docker run -it \
  $REMOVE_CONTAINER \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  --network=host \
  $DOCKER_OPTS \
  $IMG

# Put X server permissions back to what they were.  If this script is killed
# uncleanly, then this may not run.
xhost - local: