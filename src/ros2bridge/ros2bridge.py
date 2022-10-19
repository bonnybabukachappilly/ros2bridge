"""
WS Server.

start_ioloop:
    Start IO Loop.
shutdown_ioloop:
    Stop IO Loop.
bridge:
    Complete ROS websocket bridge.
"""

import signal
from typing import Any

import rclpy
from rclpy.node import Node

from ros2bridge.server.ws_server import WebSocketServer
from ros2bridge.utils.settings import get_ip, get_tornado_settings

from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.netutil import bind_sockets
from tornado.web import Application


def start_ioloop() -> None:
    """Start the loop."""
    IOLoop.instance().start()


def shutdown_ioloop(server: HTTPServer) -> None:
    """Stop the server loop.

    Args:
        server (HTTPServer): Server.
    """
    io_loop = IOLoop.instance()

    def stop_handler(*_) -> None:
        """Stop different signals."""
        def shutdown() -> None:
            """Stop the individual signals."""
            server.stop()

            def stop_loop() -> None:
                """Stop the loop."""
                io_loop.stop()
            stop_loop()
        io_loop.add_callback(shutdown)

    signal.signal(signal.SIGQUIT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)


def bridge(args: Any = None) -> None:
    """Start ROS WS bridge.

    Args:
        args (Any, optional): _description_. Defaults to None.
    """
    rclpy.init(args=args)

    RosWSBridge = Node('ros_bridge_ws')

    endpoints = [(r'/', WebSocketServer)]

    ws_app = Application(  # type: ignore [arg-type]
        endpoints,  # type: ignore [arg-type]
        **get_tornado_settings()  # type: ignore [arg-type]
    )
    url = get_ip()

    sockets = bind_sockets(**url)  # type: ignore [arg-type]
    server = HTTPServer(ws_app)
    server.add_sockets(sockets)

    try:
        ros_node = PeriodicCallback(
            lambda: rclpy.spin_once(  # type: ignore [no-any-return]
                RosWSBridge, timeout_sec=0.01
            ),
            1
        )

    except KeyboardInterrupt:
        print('Terminating WS')

    shutdown_ioloop(server)

    try:
        print(f"Websocket started: 'ws://{url['address']}:{url['port']}'")
        ros_node.start()
        start_ioloop()
    except KeyboardInterrupt:
        print('Terminating WS')

    RosWSBridge.destroy_node()
    rclpy.shutdown()
    ros_node.stop()
