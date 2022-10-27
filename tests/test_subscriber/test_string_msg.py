"""Testing ROS2 String message for subscriber."""

import json
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from websocket._core import WebSocket


# ******************************
# ******** TEST SUPPORT ********
# ******************************

_topic = '/testing/string'
_type = 'std_msgs/String'
_message = 'Hello World'


def publish_message() -> None:
    """Timer callback to publish message for testing."""
    global _publisher, _message

    msg = String()
    msg.data = _message

    _publisher.publish(msg=msg)


def publisher_thread() -> None:
    """Thread to handle publishing of message."""
    global _node, STOP_THREAD

    while not STOP_THREAD:
        rclpy.spin_once(_node)
        sleep(.5)


def publish_data() -> None:
    """Create a publisher for testing."""
    global _node, _publisher, _pub_thread, _topic

    _publisher = _node.create_publisher(
        msg_type=String,
        topic=_topic,
        qos_profile=10
    )

    _node.create_timer(
        timer_period_sec=1.0,
        callback=publish_message
    )

    _pub_thread = Thread(target=publisher_thread)
    _pub_thread.start()


def subscribe(deactivate=False) -> None:
    """Send a message via ws for testing.

    Args:
        deactivate (bool, optional): Unsubscribing. Defaults to False.
    """
    global _websocket, _topic, _type

    subscriber = {
        'operation': 'subscribe',
        'topic': _topic,
        'type': _type,
    }

    if deactivate:
        subscriber['unsubscribe'] = True

    _websocket.send(json.dumps(subscriber))


def get_message() -> dict:
    """Get message from websocket for testing.

    Returns:
        dict: Message received from websocket.
    """
    global _websocket, _topic

    _count = 10

    while True:
        _recv: dict = json.loads(_websocket.recv())
        _count -= 1

        if _recv.get('topic') == _topic:
            break

        if _count < 0:
            assert False
        sleep(1)

    return _recv


# ******************************
# ************ TEST ************
# ******************************

def test_subscriber_count(ros_node: Node, ws_conn: WebSocket) -> None:
    """Test new subscriber is added.

    Args:
        ros_node (Node): ROS Node
        ws_conn (WebSocket): Websocket connection.
    """
    global _node, _websocket, STOP_THREAD, _topic, _type

    STOP_THREAD = False
    _node = ros_node
    _websocket = ws_conn

    publish_data()
    subscribe()
    sleep(2)

    assert _node.count_subscribers(_topic) == 1


def test_subscriber_type() -> None:
    """Test subscription type."""
    global _type

    msg = get_message()
    assert _type == msg.get('type')


def test_subscriber_message() -> None:
    """Test message from subscriber."""
    global _message

    msg = get_message()
    assert _message == msg['message']['data']


def test_unsubscribe() -> None:
    """Test for unsubscribe."""
    global _node, _topic, STOP_THREAD

    subscribe(deactivate=True)
    sleep(2)

    STOP_THREAD = True

    assert _node.count_subscribers(_topic) == 0
