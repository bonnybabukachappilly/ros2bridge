"""Testing Turtlebot AMCL message for subscriber."""

from tests.utils.subscriber_helper import SubscribeTurtleBot

_topic = '/amcl_pose'
_type = 'geometry_msgs/PoseWithCovarianceStamped'

subscribe = SubscribeTurtleBot(
    msg_topic=_topic,
    msg_type=_type
)


def test_message(ws_conn) -> None:
    """Test message keys."""
    subscribe.set_websocket = ws_conn
    subscribe.ws_subscribe()
    _keys = ['header', 'pose']

    _msg = subscribe.get_message(old_data=True)

    for key in _keys:
        assert _msg.get(key) is not None


def test_header() -> None:
    """Test header message keys."""
    _msg = subscribe.get_message(old_data=True, keys=['header'])
    _keys = ['stamp', 'frame_id']

    for key in _keys:
        assert _msg.get(key) is not None


def test_header_stamp() -> None:
    """Test header message key stamp."""
    _msg = subscribe.get_message(old_data=True, keys=['header', 'stamp'])
    _keys = ['sec', 'nanosec']

    for key in _keys:
        assert _msg.get(key) is not None


def test_pose() -> None:
    """Test pose message key."""
    _msg = subscribe.get_message(old_data=True, keys=['pose'])
    _keys = ['pose', 'covariance']

    for key in _keys:
        assert _msg.get(key) is not None


def test_pose_pose() -> None:
    """Test pose message key pose."""
    _msg = subscribe.get_message(old_data=True, keys=['pose', 'pose'])

    _keys = ['position', 'orientation']

    for key in _keys:
        assert _msg.get(key) is not None
