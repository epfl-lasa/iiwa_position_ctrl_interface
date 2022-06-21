ARG BASE_IMAGE=ghcr.io/aica-technology/ros-ws:noetic
FROM ${BASE_IMAGE}


# Add the user to the current GID of the host to avoid permisson issues in volumes
ARG HOST_GID=1000
USER root
RUN groupadd --gid ${HOST_GID} host_group
RUN usermod ${USER} -g ${HOST_GID}
RUN usermod ${USER} -a -G ${USER}
USER ${USER}

WORKDIR /tmp
RUN git clone -b v6.0.0 https://github.com/epfl-lasa/control_libraries.git \
    && cd control_libraries/source && sudo ./install.sh -y --no-dynamical-systems --no-controllers

WORKDIR ${HOME}