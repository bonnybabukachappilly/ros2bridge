"""Test WS Connection."""

from time import sleep

from rclpy.node import Node

from tests.utils.pre_post import PrePost


pre_post = PrePost()


def test_connection(ws_conn) -> None:
    """Test websocket connection."""
    assert ws_conn.connected


def test_node_name(ros_node) -> None:
    """Test client node."""
    node: Node = ros_node
    _name = pre_post.get_constants('node_name')
    sleep(1)
    _nodes = node.get_node_names()
    assert _name in _nodes
