ARG ROS_DISTRO=jazzy
ARG DEBIAN_FRONTEND=noninteractive

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base

LABEL version="1.0"
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN apt update && apt -y upgrade
RUN apt-get update && apt-get install -y --no-install-recommends curl git python3-pip \
    libfuse2 qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev

RUN mkdir -p /colcon_ws/src/thirdparty
WORKDIR /colcon_ws/src/thirdparty
COPY dependencies.repos .
RUN vcs import < dependencies.repos

WORKDIR /colcon_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

# WORKDIR /root/
# RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
#     && chmod a+x Groot2.AppImage

# WORKDIR /colcon_ws

COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

FROM base AS dev

ARG UID=1000
ARG GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends gdb gdbserver vim

# remove default user
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu
RUN userdel -rf ubuntu

RUN groupadd --gid ${GID} user \
    && useradd --uid ${GID} --gid ${UID} --create-home user \
    && echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user \
    && mkdir -p /home/user \
    && chown -R ${UID}:${GID} /home/user

RUN chown -R ${UID}:${GID} /colcon_ws/

USER user
RUN echo "source /entrypoint.sh" >> /home/user/.bashrc