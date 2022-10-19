"""Test WS Connection."""


from tests.utils.pre_post import PrePost

from websocket import WebSocket


pre_post = PrePost()


def test_connection() -> None:
    """Test websocket connection."""
    _ws_url = pre_post.get_ws_url()

    _ws = WebSocket()
    _ws.connect(_ws_url)

    assert _ws.connected
