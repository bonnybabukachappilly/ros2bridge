
from tests.utils.action_client_helper import ActionTurtleBot

_action_name = '/FollowWaypoints'
_action_type = 'nav2_msgs/FollowWaypoints'
_message = {
    "poses": [
        {
            "header": {
                "stamp": {
                    "sec": 325,
                    "nanosec": 355000000
                },
                "frame_id": "odom"
            },
            "pose": {
                "position": {
                    "x": 163.85197596133244,
                    "y": 6.408348422589976,
                    "z": 0.010167012762176242
                },
                "orientation": {
                    "x": 0.0003170579113323778,
                    "y": -0.000676868286006515,
                    "z": -0.1263032685284976,
                    "w": 0.9919913939559266
                }
            }
        },
        {
            "header": {
                "stamp": {
                    "sec": 326,
                    "nanosec": 355000000
                },
                "frame_id": "odom"
            },
            "pose": {
                "position": {
                    "x":  -0.3943325067645821,
                    "y": 0.5282717743793515,
                    "z": 0.008529553660748071
                },
                "orientation": {
                    "x": -0.001635945381023378,
                    "y": 0.002353456667572901,
                    "z": 0.5825792052694925,
                    "w": 0.8127688813629561
                }
            }
        }
    ]
}

action = ActionTurtleBot(
    action_name=_action_name,
    action_type=_action_type,
    message=_message
)


def check_message(msg: dict) -> None:
    """Test action feedback keys.

    Args:
        msg (dict): Feedback from action server.
    """
    _keys = [
        'current_pose',
        'navigation_time',
        'number_of_recoveries',
        'distance_remaining'
    ]

    _msg = msg

    action.check_keys(_keys, _msg)


def check_current_pose(msg: dict) -> None:
    """Test action feedback keys.

    Args:
        msg (dict): Feedback from action server.
    """
    _keys = ['header', 'pose']
    _filter = ['current_pose']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)

    # checking header
    _keys = ['stamp', 'frame_id']
    _filter = ['current_pose', 'header']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)

    # checking header.stamp
    _keys = ['sec', 'nanosec']
    _filter = ['current_pose', 'header', 'stamp']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)

    # checking pose
    _keys = ['position', 'orientation']
    _filter = ['current_pose', 'pose']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)

    # checking pose.position
    _keys = ['x', 'y', 'z']
    _filter = ['current_pose', 'pose', 'position']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)

    # checking pose.orientation
    _keys = ['x', 'y', 'z', 'w']
    _filter = ['current_pose', 'pose', 'orientation']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)


def check_navigation_time(msg: dict) -> None:
    """Test action feedback keys.

    Args:
        msg (dict): Feedback from action server.
    """
    _keys = ['sec', 'nanosec']
    _filter = ['navigation_time']
    _msg = action.get_by_key(data=msg, keys=_filter)
    action.check_keys(_keys, _msg)


def test_response(ws_conn) -> None:
    """Test for action response."""
    action.set_websocket = ws_conn
    action.create_call_service()
    action.collect_action_response()

    messages: list = action.get_action_response('response')
    for message in messages:
        assert message.get('message') == 'Goal Accepted.'


def test_feedback() -> None:
    """Test for action feedback."""
    messages: list = action.get_action_response('feedback')
    for message in messages:
        check_message(message.get('message'))
        check_current_pose(message.get('message'))
        check_navigation_time(message.get('message'))
