"""Testing action client navigation for Action Client."""


from tests.utils.action_client_helper import ActionTurtleBot


_action_name = '/navigate_to_pose'
_action_type = 'nav2_msgs/NavigateToPose'
_message = {
    'pose': {
        'header': {
            'stamp': {
                'sec': 13,
                'nanosec': 401000000,
            }
        },
        'pose': {
            'position': {
                'x': 0.05,
                'y': -0.005,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': -0.0038,
                'w': 0.999
            }
        }
    }
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


def test_result() -> None:
    """Test for action result."""
    messages: list = action.get_action_response('result')
    for message in messages:
        assert message.get('message').get('result') is not None
