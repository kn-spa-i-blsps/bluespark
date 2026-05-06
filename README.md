# BlueSpark

BlueSpark is an AUV (autonomous underwater vehicle) project designed for a wide variety of technical tasks, from seafloor mapping and underwater infrastructure inspection, through environmental data collection, to rare metal prospecting, ultimately intended for multi-robot operation.

This repository contains the code responsible for the robot's control and missions. For more information about the entire project, not just the code, check out the [project website](https://knspaiblsps.pl/project.html?id=bluespark).

Codebase and information about research and development of BlueSpark's *computer vision* can be found [here](https://github.com/MaciejBorowiecki/BlueSpark-autonomy).

# Prerequisites
The recommended and easiest way to run the BlueSpark codebase is using Docker. This ensures a consistent environment and eliminates the need for complex local setup.

## Host Requirements
Docker installed on your machine.

## Environment Stack
The Docker image is pre-configured with the following stack. If you choose to build the project locally without Docker, ensure you have these dependencies installed:

- **Framework**: ROS 2 Humble
- **Build System**: colcon (python3-colcon-common-extensions)
- **Python**: Python 3 and pip
- **System Libraries**: libgl1-mesa-glx (required for OpenGL/graphical interfaces)
- **Python Packages**: Specific Python dependencies are listed in the `requirements.txt` file.

# Packages 

## bluespark_vision

This package contains the core **vision node** and supporting perception modules responsible for real-time **object detection**, **distance, and angle estimation**. The main vision_node processes camera frames and publishes the 3D position, camera angles, and rotation of detected targets. It utilizes a YOLO-based object detector, a universal camera interface (supporting both USB and Raspberry Pi CSI cameras), and a distance calculator for mathematical pose estimation.

## bluespark_interfaces

This package contains custom ROS 2 interfaces, currently supporting features such as:

#### Custom messages
- `DetectedObject` - specific information about detected objects.
    - `DetectedObjectArray` - an array of `DetectedObject` messages, published together in a single message for efficiency and convenience.


## bluespark_control

In this package there are nodes rosposible for low level control over mission of the BlueSpark robot. Their main tasks are: arming (or disarming), status checking (connected or disconnected) and mode setting. To do so they mostly use services launched by mavros, so before using any node from those: **SetModeNode**, **StateListener**,
**ArmingNode** make sure that the mavros is launched.

Furthermore, in this package there is also **RCOverrideNode**, responsible for sending RC Override channels to the engines. In fact, the node simulates the RC controler in hand of a pilot. It lanuches services of all available degrees of freedom for other nodes to call.

#### Custom services
- `SetRCOverride` - setting override using OverrideRCIn mavros service.