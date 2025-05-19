# Ros Bridge POC <!-- omit in toc -->

## Description <!-- omit in toc -->

This is a project to test ROS Bridge.

## Table of Contents <!-- omit in toc -->

- [Prerequisites](#prerequisites)
- [Checkout and build the project](#checkout-and-build-the-project)
- [Known issue with roslibpy](#known-issue-with-roslibpy)
- [Troubleshooting](#troubleshooting)
  - [No executable found](#no-executable-found)

## Prerequisites

We need a computer with Ubuntu 22.04+ and Docker installed. The following process will create a Docker container with use "developer" and password "developer".

## Checkout and build the project

Open a terminal and run

```bash
git clone --recurse-submodules git@github.com:kineticsystem/ros-brigde-poc.git
```

Move into the repo folder and type:

```bash
./docker/dock.sh bridge build
./docker/dock.sh bridge start
```

When the docket container starts, you can type:

```bash
pip install -r requirements.txt
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --event-handlers log-
```

On the same terminal, you can run the server:

```bash
source install/setup.bash
ros2 launch bridge server.launch.py
```

To run the action client, open a new terminal and type

```bash
./docker/dock.sh bridge start
```

Then run the rclpy client with:

```bash
source install/setup.bash
ros2 run bridge rclpy_client.py
```

Or, you can run the roslibpy client with:

```bash
source install/setup.bash
ros2 run bridge roslib_client.py
```

## Known issue with roslibpy

The client with rclpy runs perfectly. The client with roslibpy fails, with the server throwing the following exception:

```bash
[rosbridge_websocket-3] [ERROR] [1747684611.861033299] [rosbridge_websocket]: [Client 7e715482-068a-4a2f-a734-4f5eaa2d6c8a] [id: subscribe:/test_action/feedback:4] subscribe: Unable to import bridge_msgs.msg from package bridge_msgs. Caused by: No module named 'bridge_msgs.msg'
```

Ros Bridge looks for the action in the wrong package, msg instead of action.

## Troubleshooting

### No executable found

```bash
ros2 run bridge bridge_server.py
No executable found
```

Check if the Python file is executable (+x flag).
