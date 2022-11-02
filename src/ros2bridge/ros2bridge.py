"""ROS 2 BRIDGE.

This is the main function responsible for creating a bridge between
ROS 2 DDS and Websocket.

Methods:
    start_ioloop() -> None:
        Start the io loop.

    shutdown_ioloop(server: HTTPServer) -> None:
        Stop the io loop.

    bridge(args: Namespace) -> None:
        Start ROS WS bridge.
"""

import signal
from argparse import Namespace

import rclpy

from ros2bridge.ros.bridge_node import RosWSBridge
from ros2bridge.server.ws_server import WebSocketServer
from ros2bridge.utils.settings import get_ip, get_tornado_settings

from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.netutil import bind_sockets
from tornado.web import Application


def start_ioloop() -> None:
    """Start the io loop."""
    IOLoop.instance().start()


def shutdown_ioloop(server: HTTPServer) -> None:
    """Stop the io loop.

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


def bridge(args: Namespace) -> None:
    """Start ROS WS bridge.

    Args:
        args (Namespace): Argument from argparse.
    """
    rclpy.init()

    ros_ws_bridge = RosWSBridge()

    endpoints = [(r'/', WebSocketServer)]

    ws_app = Application(  # type: ignore [arg-type]
        endpoints,  # type: ignore [arg-type]
        **get_tornado_settings()  # type: ignore [arg-type]
    )

    url = get_ip(port=str(args.port), ngrok=bool(args.ngrok))

    sockets = bind_sockets(**url)  # type: ignore [arg-type]
    server = HTTPServer(ws_app)
    server.add_sockets(sockets)

    ros_node = PeriodicCallback(
        lambda: rclpy.spin_once(  # type: ignore [no-any-return]
            ros_ws_bridge, timeout_sec=0.01
        ),
        1
    )

    shutdown_ioloop(server)

    try:
        print(
            f"Websocket started: 'ws://{url['address']}:{url['port']}'")
        ros_node.start()
        start_ioloop()
    except KeyboardInterrupt:
        print('Terminating WS')

    ros_ws_bridge.destroy_node()
    rclpy.shutdown()
    ros_node.stop()
