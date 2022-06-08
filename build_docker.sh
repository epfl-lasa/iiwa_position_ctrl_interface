#!/bin/bash

# Name and base
IMAGE_NAME=epfl-lasa/iiwa_position_ctrl_interface
BASE_IMAGE=ghcr.io/aica-technology/ros-ws:noetic

# What does that do?
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
if [[ ! -f "${SCRIPT_DIR}"/config/sshd_entrypoint.sh ]]; then
  mkdir -p "${SCRIPT_DIR}"/config
  cp "$(dirname "${SCRIPT_DIR}")"/common/sshd_entrypoint.sh "${SCRIPT_DIR}"/config/ || exit 1
fi

# Help
HELP_MESSAGE="Usage: ./build.sh [-r, --rebuild] [-v, --verbose] [-i, --image-name]
Build the '${IMAGE_NAME}' image.
Options:
  -r, --rebuild            Rebuild with --no-cache option.
  -v, --verbose            Display additional info upon build.
  -i, --image-name         Defines the name given to the image beeing built.
  -h, --help               Show this help message."

# Parse build flags
BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
  -r | --rebuild)
    BUILD_FLAGS+=(--no-cache)
    shift 1
    ;;
  -v | --verbose)
    BUILD_FLAGS+=(--progress=plain)
    shift 1
    ;;
-i | --image-name)
    IMAGE_NAME=$2
    shift 2
    ;;
-h | --help)
    echo "${HELP_MESSAGE}"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done


# Try to pull image
docker pull "${BASE_IMAGE}" || echo -e "\'033[33mCould not pull docker image ${BASE_IMAGE}"

# Setup build flags
BUILD_FLAGS+=(--build-arg BASE_IMAGE="${BASE_IMAGE}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .