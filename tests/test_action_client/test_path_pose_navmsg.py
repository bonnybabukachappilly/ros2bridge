"""Testing action client Compute path to pose for Action Client."""


from tests.utils.action_client_helper import ActionTurtleBot


_action_name = '/compute_path_to_pose'
_action_type = 'nav2_msgs/ComputePathToPose'
_message = {
    '_pose': {
        '_header': {
            '_stamp': {
                '_sec': 13,
                '_nanosec': 401000000
            },
        },
        '_pose': {
            '_position': {
                '_x': 0.15,
                '_y': -0.005,
                '_z': 0.0
            },
            '_orientation': {
                '_x': 0.0,
                '_y': 0.0,
                '_z': -0.0038,
                '_w': 0.999
            }
        }
    },
    '_planner_id': "< class 'str' >"
}


action = ActionTurtleBot(
    action_name=_action_name,
    action_type=_action_type,
    message=_message
)


def test_response(ws_conn) -> None:
    """_summary_.

    Args:
        ws_conn (_type_): _description_
    """
    action.set_websocket = ws_conn
    action.create_call_service()
    action.collect_action_response()

    messages: list = action.get_action_response('response')
    for message in messages:
        assert message.get('message') == 'Goal Accepted.'


def test_result() -> None:
    """Test for action result."""
    message: list = action.get_action_response('result')[0].get('message')

    _keys = ['path', 'planning_time']

    action.check_keys(_keys, message)


def test_result_path() -> None:
    """Test for action result keys."""
    message: list = action.get_action_response('result')[0].get('message')

    _keys = ['header', 'poses']
    _filter = ['path']

    _msg = action.get_by_key(data=message, keys=_filter)

    action.check_keys(_keys, _msg)


def test_result_path_header() -> None:
    """Test for action result keys."""
    message: list = action.get_action_response('result')[0].get('message')

    _keys = ['stamp', 'frame_id']
    _filter = ['path', 'header']

    _msg = action.get_by_key(data=message, keys=_filter)

    action.check_keys(_keys, _msg)


def test_result_path_header_stamp() -> None:
    """Test for action result keys."""
    message: list = action.get_action_response('result')[0].get('message')

    _keys = ['sec', 'nanosec']
    _filter = ['path', 'header', 'stamp']

    _msg = action.get_by_key(data=message, keys=_filter)

    action.check_keys(_keys, _msg)


def test_result_planning_time() -> None:
    """Test for action result keys."""
    message: list = action.get_action_response('result')[0].get('message')

    _keys = ['sec', 'nanosec']
    _filter = ['planning_time']

    _msg = action.get_by_key(data=message, keys=_filter)

    action.check_keys(_keys, _msg)
