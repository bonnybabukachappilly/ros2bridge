"""
pytest fixture for testing.

contextmanager:
    create_ros_node: Create a ROS node.

fixture:
    ws_conn: Create a websocket connection for testing.
    ros_node: Create a ROS node for testing.
"""


from contextlib import closing, contextmanager
from typing import Generator

import pytest


import rclpy
from rclpy.node import Node

from tests.utils.pre_post import PrePost

from websocket import create_connection
from websocket._core import WebSocket


pre_post = PrePost()


@contextmanager
def create_ros_node(node_name: str) -> Generator[Node, None, None]:
    """Create a ROS node.

    Args:
        node_name (str): Name of the node.

    Yields:
        Generator[Node, None, None]: Created node.
    """
    rclpy.init()
    node = Node(node_name=node_name)

    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope='session')
def ws_conn() -> Generator[WebSocket, None, None]:
    """Create a websocket connection for testing.

    Yields:
        Generator[WebSocket, None, None]: Created websocket.
    """
    url = pre_post.get_ws_url()
    with closing(create_connection(url)) as ws:
        yield ws


@pytest.fixture(scope='session')
def ros_node() -> Generator[Node, None, None]:
    """Create a ROS node for testing.

    Yields:
        Generator[Node, None, None]: Created ROS node.
    """
    with create_ros_node('ws_bridge_testing_node') as node:
        yield node
