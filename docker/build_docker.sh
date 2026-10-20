#!/bin/bash
# The build context is the repository root: the image builds the checked-out ergocub_navigation.
cd "$(dirname "$0")/.."
docker build . --build-arg "GIT_USERNAME=$1" --build-arg "GIT_USER_EMAIL=$2" -t ergocub_navigation:jazzy -f docker/Dockerfile_jazzy
