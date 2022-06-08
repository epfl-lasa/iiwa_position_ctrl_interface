ARG BASE_IMAGE=ghcr.io/aica-technology/ros-ws:noetic
FROM ${BASE_IMAGE}

WORKDIR /tmp
RUN git clone -b v4.0.0 https://github.com/epfl-lasa/control_libraries.git \
    && cd control_libraries/source && sudo ./install.sh -y
