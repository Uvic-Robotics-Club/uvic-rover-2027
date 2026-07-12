#########################################################
# Development Image
#########################################################

# Full ROS 2 Jazzy desktop image on Ubuntu 24.04.
# Used for local development, including GUI tools like RViz/rqt.
FROM osrf/ros:jazzy-desktop AS dev

# Avoid interactive prompts during apt installs.
ARG DEBIAN_FRONTEND=noninteractive

# Basic dev tools + C++ build tooling + ROS 2 build tooling
#
# nano / vim: terminal text editors.
# curl: downloads files/data from URLs.
# git / git-lfs: version control, including large file support.
# python3-pip: installs Python packages from requirements.txt.
# build-essential: compilers and build tools (gcc, g++, make).
# cmake: build system used by ament_cmake / C++ ROS 2 packages.
# gdb: debugger for C++ nodes.
# clang-format: C++ code formatting.
# lsb-release / gnupg2: needed for managing apt repo keys.
# sudo: allows the non-root dev user to run admin commands.
# x11-apps: GUI test tools like `xeyes`.
# mesa-utils: OpenGL test/debug tools like `glxinfo` and `glxgears`.
# libgl1-mesa-glx: OpenGL libraries needed by GUI tools such as RViz.
# python3-colcon-common-extensions: colcon build tool for ROS 2 workspaces.
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    curl \
    git \
    git-lfs \
    python3-pip \
    build-essential \
    cmake \
    gdb \
    clang-format \
    lsb-release \
    gnupg2 \
    sudo \
    x11-apps \
    mesa-utils \
    libgl1 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies used by ROS Python nodes.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

# Create a non-root user for normal development.
# This avoids editing/building everything as root inside the container.
# The base image may already have a user/group at UID/GID 1000 (e.g. "ubuntu"),
# so we reuse it if it exists instead of failing on a collision.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if getent group $USER_GID > /dev/null 2>&1; then \
        EXISTING_GROUP=$(getent group $USER_GID | cut -d: -f1); \
        groupmod -n $USERNAME $EXISTING_GROUP; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
    fi \
    && if getent passwd $USER_UID > /dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        usermod -l $USERNAME -d /home/$USERNAME -m $EXISTING_USER; \
        usermod -s /bin/bash $USERNAME; \
    else \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && mkdir -p /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Add the dev user to hardware-access groups.
# dialout: allows access to serial devices like /dev/ttyUSB0 and /dev/ttyACM0.
# plugdev: commonly used for removable/plugged-in device permissions.
RUN usermod -aG dialout,plugdev $USERNAME

# Configure every new terminal for ROS 2 development.
# This sources ROS 2 Jazzy, sources the workspace when it exists,
# and wraps colcon build so a successful build is immediately sourced
# in the current terminal.
RUN { \
        echo ""; \
        echo "# ROS 2 auto-setup"; \
        echo "if [ -f /opt/ros/jazzy/setup.bash ]; then"; \
        echo "  source /opt/ros/jazzy/setup.bash"; \
        echo "fi"; \
        echo ""; \
        echo "if [ -f /rover_ws/install/setup.bash ]; then"; \
        echo "  source /rover_ws/install/setup.bash"; \
        echo "fi"; \
        echo ""; \
        echo "# Wrapper: after colcon build succeeds, source the workspace in this terminal"; \
        echo "colcon() {"; \
        echo "  command colcon \"\$@\""; \
        echo "  local status=\$?"; \
        echo "  if [ \"\$1\" = \"build\" ] && [ \$status -eq 0 ] && [ -f /rover_ws/install/setup.bash ]; then"; \
        echo "    source /rover_ws/install/setup.bash"; \
        echo "  fi"; \
        echo "  return \$status"; \
        echo "}"; \
    } >> /home/$USERNAME/.bashrc

# Switch to the non-root development user and create the workspace.
USER $USERNAME
WORKDIR /rover_ws

# Start an interactive shell by default.
CMD ["/bin/bash"]