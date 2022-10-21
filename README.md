# ROS WEBSOCKET BRIDGE

![version](https://img.shields.io/badge/Version-ROS%202%20FOXY-informational)
![license](https://img.shields.io/badge/license-GNU%20v3.0-blue)

## Overview

This is a package for converting ROS 2 DDS to websocket. This project was inspired by [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) by [Robo Web Tools](https://github.com/RobotWebTools). The implemented features are not for specific topics or services but all available ros interfaces. It's in early development, and not all interfaces are tested. Any bug reported will be fixed in upcoming releases.

When using the suite, I faced issues with ROS 2 and couldn't find a way to move forward. So I decided to create another which is not advanced as the suite but gets my job done. I decided to package it and publish it on PyPI for easy availability to others and learning purposes. The code I use is different, and the features are not the same. The client libraries for the suite won't work because this is not a suite clone.

**Note: This project is not a suite clone. The architecture and implementation are different. Not all the features of the suite are not available in this project.**

## Requirements

----

* Ubuntu 20.04
* Python 3.8
* ROS 2 Foxy

## Installation

----

```bash
pip3 install ros2bridge
```

## Usage

----

```bash
python3 -m ros2bridge

```

**optional parameters:**

* -p, --port : For specific port. Default is 9020
* -n, --ngrok : By default, websocket is hosted on local-IP. If this flag is set ws is hosted internally

**NOTE: for using custom ros interface source that workspace before running the bridge.**

## Available ROS Interfaces

----

* Publisher

**TODO :**

* Subscriber
* Service Client
* Action Client

## Request structure

----

### Publisher

```json
{
    "operation": "publish",
    "topic": "/<topic_name>",
    "type": "<message_parent>/<message_type>",
    "message": "<message>"
}
```

* type eg: 'std_msgs/String'
* message eg: {"data" : "HELLO WORLD !"}
* NOTE: message should be in the format of message type

----
