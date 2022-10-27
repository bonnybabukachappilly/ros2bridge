"""Testing ROS2 Twist message for publisher."""

import json
from threading import Thread
from time import sleep
from typing import Any, Dict

from geometry_msgs.msg import Twist
from geometry_msgs.msg._twist import Twist as twist

from rclpy.node import Node

from websocket._core import WebSocket

# ******************************
# ******** TEST SUPPORT ********
# ******************************

_topic = '/testing/cmd_vel'
_type = 'geometry_msgs/Twist'
_message = {
    'linear': {
        'x': 0.5,
    },
    'angular': {
        'z': 0.5
    }
}


def publish_data(data: Dict[str, Any]) -> None:
    """Publish message via websocket for testing.

    Args:
        data (Dict[str, Any]): Message to be send via ws.
    """
    global STOP_THREAD, _websocket

    while not STOP_THREAD:
        _websocket.send(json.dumps(data))
        sleep(2)


def create_publisher():
    """Create publisher for testing."""
    global _pub_thread, _topic, _type, _message

    publisher = {
        'operation': 'publish',
        'topic': _topic,
        'type': _type,
        'message': _message
    }

    _pub_thread = Thread(target=publish_data, args=(publisher,))
    _pub_thread.start()

    sleep(2)


def get_topic_info() -> list:
    """Get topic info by topic name.

    Returns:
        list: Topic info as a list.
    """
    global _node, _topic
    sleep(2)
    return _node.get_publishers_info_by_topic(_topic)


# ******************************
# ************ TEST ************
# ******************************


def test_publisher_topic_exist(ws_conn: WebSocket, ros_node: Node) -> None:
    """Test the publisher topic.

    Args:
        ws_conn (WebSocket): _description_
        ros_node (Node): _description_
    """
    global STOP_THREAD, _websocket, _node

    _websocket = ws_conn
    _node = ros_node
    STOP_THREAD = False

    create_publisher()
    _running_publisher = get_topic_info()

    assert len(_running_publisher) > 0


def test_publisher_topic_name() -> None:
    """Test message type of the publisher."""
    global _type

    _running_publisher = get_topic_info()

    _running_publisher = _running_publisher[0]
    __type = _type.split('/')

    assert _running_publisher._topic_type == f'{__type[0]}/msg/{__type[1]}'


def test_publisher_message() -> None:
    """Test the published message."""
    global _node, _type, _topic, _message, STOP_THREAD

    def sub_callback(msg: twist):
        """Sub callback.

        Args:
            msg (twist): Message received from subscription.
        """
        assert msg.linear.x == _message['linear']['x']
        assert msg.angular.z == _message['angular']['z']

    subscriber = _node.create_subscription(
        Twist,
        _topic,
        sub_callback,
        10
    )

    subscriber.destroy()

    STOP_THREAD = True
