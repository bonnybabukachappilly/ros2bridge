"""Testing Turtlebot AMCL for Service Client."""

from tests.utils.service_client_helper import ServiceTurtleBot

_srv_name = '/amcl/get_state'
_srv_type = 'lifecycle_msgs/GetState'
_message = {}


srv = ServiceTurtleBot(
    srv_name=_srv_name,
    srv_type=_srv_type,
    message=_message
)


def test_structure(ws_conn) -> None:
    """_summary_.

    Args:
        ws_conn (_type_): _description_
    """
    srv.set_websocket = ws_conn
    srv.create_call_service()

    _keys = ['id', 'label']
    _filter = ['current_state']

    _msg = srv.get_message(old_data=True, keys=_filter)

    for key in _keys:
        assert _msg.get(key) is not None
