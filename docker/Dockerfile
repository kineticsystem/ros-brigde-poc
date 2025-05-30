# Install Ubuntu 20.04 with user "developer".

FROM osrf/ros:humble-desktop-full-jammy as ros

LABEL name="develop:ros"

RUN apt update
RUN apt -y install sudo
RUN echo 'root:docker' | chpasswd                      # Update user root password.

RUN groupadd -g 107 input                              # Create the group 'input' (id 107) to access devices.

RUN useradd --create-home --shell /bin/bash developer  # Create the user developer.
RUN echo 'developer:developer' | chpasswd              # Update user developer password.
RUN usermod -aG sudo,input developer                   # Add user to sudoers and input devices.

RUN apt -y upgrade
RUN apt -y install wget
RUN apt -y install unzip
RUN apt -y install python3
RUN apt -y install python-is-python3
RUN apt install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp
RUN apt install pip -y

CMD ["bash"]

USER developer

RUN mkdir -p /home/developer/.ros
COPY --chown=developer:developer cyclonedds.xml /home/developer/.ros

ENV HOME /home/developer
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=~/.ros/cyclonedds.xml" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN rosdep update

WORKDIR /home/developer/ws
