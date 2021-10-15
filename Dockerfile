# Base Image
FROM ubuntu:20.04

RUN useradd -ms /bin/bash komatsu
SHELL ["/bin/bash", "-c"]

# Set any environment
ENV CC=/usr/bin/clang-10
ENV CXX=/usr/bin/clang++-10
ENV CPP=/usr/bin/clang-cpp-10

WORKDIR /usr/src/ros_ws/src

# APT Update to get current locations.
RUN apt-get update --assume-yes && apt-get upgrade --assume-yes

RUN DEBIAN_FRONTEND="noninteractive" apt-get install --assume-yes \
  clang-10 \
  gpg \
  wget \
  git \
  cmake \
  locales \
  curl \
  gnupg \
  lsb-release

# Set Locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages.
RUN apt-get update && apt-get install --assume-yes ros-galactic-desktop python3-colcon-common-extensions python3-rosdep2

# Install Gaia.
RUN curl --limit-rate 1G -o /home/komatsu/gaia-0.3.1_amd64.deb \
      https://gaia-sdk.s3.us-west-2.amazonaws.com/private-releases/0.3.1-beta/gaia-0.3.1_amd64.deb && \
    apt install -y /home/komatsu/gaia-0.3.1_amd64.deb && \
    chown komatsu:komatsu /home/komatsu/gaia-0.3.1_amd64.deb

# Switch to user komatsu.
USER komatsu
RUN mkdir -p /home/komatsu/ros_ws/src
WORKDIR /home/komatsu/ros_ws

RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc && rosdep update

# Checkout ROS-TCP-Endpoint & rosbag2_snapshot.
RUN cd src && \
    git clone -b ROS2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git  && \
    git clone https://github.com/gaia-platform/rosbag2_snapshot.git

# Build
RUN source /opt/ros/galactic/setup.bash && colcon build && echo 'source ~/ros2_ws/install/local_setup.bash' >> ~/.bashrc

# Copy the danger_zone project.
COPY Ros/src/* src/

# rosdep needs
USER root
RUN source /opt/ros/galactic/setup.bash && rosdep update && rosdep install -y -i --from-paths .

USER komatsu
RUN source /opt/ros/galactic/setup.bash && colcon build


# Unless overridden, this is the command line that will be executed
# when the container is run.
# CMD ["node", "agent.js"]
