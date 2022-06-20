#!/bin/bash

IMAGE_NAME="epfl-lasa/iiwa_position_ctrl_interface"
CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
USERNAME="ros"
MODE=()


# Help
HELP_MESSAGE="Usage: ./start_dockers.sh [interactive | server | connect] [-i, --image] [-u, --user]
Build the '${IMAGE_NAME}' image.
Options:
  interactive            Spin the image in the console
  server                 Spin the image as an ssh server
  connect                Connects to an active container
  -i, --image            The name of the image to use to start the container
  -u, --user             Specify the name of the login user. (optional)
  -h, --help             Show this help message and the one from aica-docker
  Additional arguments are passed to the aica-docker command.
  "

# Argument parsing
RUN_FLAGS=()
FWS_FLAGS=()
SHOW_HELP=false

while [ "$#" -gt 0 ]; do
    case "$1" in
    -i | --image)
        IMAGE_NAME=$2
        shift 2
        ;;
    -u | --user)
        USERNAME=$2
        shift 2
        ;;
    -m | --mode)
        MODE=$2
        shift 2
        ;;
    -h | --help)
        SHOW_HELP=true
        shift 1
        ;;
    *)
        if [ -z "${MODE}" ]; then
            MODE=$1
        else
            FWD_ARGS+=("$1")
        fi
        shift 1
        ;;
    esac
done

if $SHOW_HELP; then
    echo $HELP_MESSAGE
    aica-docker $MODE -h
    exit 1
fi


# Create shared volumes to work with
if [ "${MODE}" != "connect" ]; then
    
    # ROS postion control package
    docker volume rm iiwa_position_ctrl_ros_pckg
    docker volume create --driver local \
        --opt type="none" \
        --opt device="${PWD}/iiwa_position_ctrl" \
        --opt o="bind" \
        "iiwa_position_ctrl_interface_vol"
    FWD_ARGS+=(--volume="iiwa_position_ctrl_interface_vol:/home/${USERNAME}/ros_ws/iiwa_position_ctrl")

    # ROS positon control messages package
    docker volume rm iiwa_position_ctrl_ros_pckg
    docker volume create --driver local \
        --opt type="none" \
        --opt device="${PWD}/iiwa_position_msgs" \
        --opt o="bind" \
        "iiwa_position_ctrl_interface_vol"
    FWD_ARGS+=(--volume="iiwa_position_ctrl_interface_vol:/home/${USERNAME}/ros_ws/iiwa_position_msgs")

    # If a vscode file exist, add it to the ROS ws to help linting
    if [ -d "${PWD}/.vscode" ]; then
        docker volume rm iiwa_position_ctrl_vscode_param
        docker volume create --driver local \
            --opt type="none" \
            --opt device="${PWD}/.vscode" \
            --opt o="bind" \
            "iiwa_position_ctrl_interface_vol"
        FWD_ARGS+=(--volume="iiwa_position_ctrl_interface_vol:/home/${USERNAME}/ros_ws/.vscode")
    fi
fi

aica-docker \
    "${MODE}" \
    "${IMAGE_NAME}" \
    -u "${USERNAME}" \
    -n "${CONTAINER_NAME}" \
    "${FWD_ARGS[@]}" \
