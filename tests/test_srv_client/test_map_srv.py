"""Testing Turtlebot Map Server for Service Client."""

from tests.utils.service_client_helper import ServiceTurtleBot


_srv_name = '/map_server/map'
_srv_type = 'nav_msgs/GetMap'
_message = {}

srv = ServiceTurtleBot(
    srv_name=_srv_name,
    srv_type=_srv_type,
    message=_message
)


def test_message(ws_conn) -> None:
    srv.set_websocket = ws_conn
    srv.create_call_service()

    _keys = ['header', 'info', 'data']
    _filter = ['map']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_header() -> None:

    _keys = ['stamp', 'frame_id']
    _filter = ['map', 'header']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_header_stamp() -> None:

    _keys = ['sec', 'nanosec']
    _filter = ['map', 'header', 'stamp']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_info() -> None:

    _keys = ['map_load_time', 'resolution', 'width', 'height', 'origin']
    _filter = ['map', 'info']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_info_map_load_time() -> None:

    _keys = ['sec', 'nanosec']
    _filter = ['map', 'info', 'map_load_time']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_info_origin() -> None:

    _keys = ['position', 'orientation']
    _filter = ['map', 'info', 'origin']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_info_origin_position() -> None:

    _keys = ['x', 'y', 'z']
    _filter = ['map', 'info', 'origin', 'position']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None


def test_info_origin_orientation() -> None:

    _keys = ['x', 'y', 'z', 'w']
    _filter = ['map', 'info', 'origin', 'orientation']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None
