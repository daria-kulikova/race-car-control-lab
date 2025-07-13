# syntax=docker/dockerfile:1
FROM ros:noetic-ros-core-focal

# Install, on top of core, the desktop packages of ROS
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1*

# install bootstrap tools
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# -------- CRS setup --------

WORKDIR /code

# APT dependencies for the compilation of casadi/acados + Python build dependencies
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    git \
    ccache \
    python3-pip \
    python3-dev \
    libeigen3-dev \
    # IPOPT for lighthouse_calib
    gfortran \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    # Python build dependencies for cffi and other packages
    libffi-dev \
    gcc \
    g++

# Compile and install CasADi
COPY .setup/ubuntu/install_casadi.sh .setup/ubuntu/install_casadi.sh
RUN sudo ./.setup/ubuntu/install_casadi.sh

# Compile and install acados
COPY .setup/ubuntu/install_acados.sh .setup/ubuntu/install_acados.sh
RUN sudo ./.setup/ubuntu/install_acados.sh
ENV ACADOS_SOURCE_DIR="/acados"
ENV LD_LIBRARY_PATH="/acados/lib:$LD_LIBRARY_PATH"

# Setup the Tera renderer on Arm machines
COPY .setup/ubuntu/install_tera_renderer.sh .setup/ubuntu/install_tera_renderer.sh
RUN sudo ./.setup/ubuntu/install_tera_renderer.sh

# Other apt dependencies
RUN \
    --mount=type=cache,sharing=locked,target=/var/lib/apt/lists \
    --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update && apt-get install -y --no-install-recommends \
    # Joystick support
    ros-noetic-joy \
    # Realitimetools for RT Controller
    ros-noetic-realtime-tools \
    # Plotjuggler
    ros-noetic-plotjuggler \
    ros-noetic-plotjuggler-ros \
    python3-tk \
    # required by Matplotlib
    libfreetype6-dev \
    # required by wifi_com
    protobuf-compiler \
    libprotobuf-dev \
    clang-format \
    # GDB debugger
    gdb \
    valgrind \
    tmux

# Upgrade pip and install base build tools first
RUN --mount=type=cache,sharing=locked,target=/root/.cache \
    python3 -m pip install --upgrade pip wheel setuptools


# Install Python requirements, caching the installation
COPY requirements.txt requirements.txt 
RUN --mount=type=cache,sharing=locked,target=/root/.cache \
    python3 -m pip install -r requirements.txt --ignore-installed

# Install catkin
RUN --mount=type=cache,sharing=locked,target=/root/.cache \
    pip3 install catkin_tools

COPY .crs/ .crs/
RUN sudo ./.crs/setup-crs.sh

# OPENMP SETTINGS
ENV OMP_NUM_THREADS=2
ENV OMP_PLACES=cores
ENV OMP_PROC_BIND=TRUE

# GTEST
COPY .setup/ubuntu/install_gtest.sh .setup/ubuntu/install_gtest.sh
RUN sudo ./.setup/ubuntu/install_gtest.sh

# Let git run pre-commit inside container
RUN git config --system --add safe.directory *

# MANIPULATE BASHRC
RUN echo "[[ -f /code/devel/setup.bash ]] && source /code/devel/setup.bash" >> ~/.bashrc