# syntax=docker/dockerfile:1
FROM ubuntu:focal

# -------- Basic ROS setup --------
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# -------- CRS setup --------

WORKDIR /code

# APT DEPENDENCIES
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ccache \
    libsasl2-dev \
    python3-pip \
    libgmp3-dev \
    libsnmp-dev \
    libeigen3-dev \
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
    tmux \
    # IPOPT for lighthouse_calib
    gfortran \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*


# INSTALL REQUIREMENTS
COPY requirements.txt requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# Install catkin
RUN pip3 install --no-cache-dir catkin_tools

COPY .crs/ .crs/
RUN sudo ./.crs/setup-crs.sh

# CASADI
COPY .setup/ubuntu/install_casadi.sh .setup/ubuntu/install_casadi.sh
RUN sudo ./.setup/ubuntu/install_casadi.sh

# ACADOS
COPY .setup/ubuntu/install_acados.sh .setup/ubuntu/install_acados.sh
RUN sudo ./.setup/ubuntu/install_acados.sh
ENV ACADOS_SOURCE_DIR="/acados"
ENV LD_LIBRARY_PATH="/acados/lib:$LD_LIBRARY_PATH"

# Setup the Tera renderer on Arm machines
COPY .setup/ubuntu/install_tera_renderer.sh .setup/ubuntu/install_tera_renderer.sh
RUN sudo ./.setup/ubuntu/install_tera_renderer.sh

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
