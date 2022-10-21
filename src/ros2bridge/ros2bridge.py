"""
WS Server.

start_ioloop:
    Start IO Loop.
shutdown_ioloop:
    Stop IO Loop.
bridge:
    Complete ROS websocket bridge.
"""

import logging as log
import logging.config
import signal
from argparse import Namespace
from os import path

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


def my_log() -> None:
    """Adding logging function to the module."""

    log_file_path = path.join(path.dirname(
        path.abspath(__file__)), 'logging.conf')

    logging.config.fileConfig(log_file_path)


def bridge(args: Namespace) -> None:
    """Start ROS WS bridge.

    Args:
        args (Namespace): Argument from argparse.
    """
    rclpy.init()

    my_log()

    RosWSBridge = Node('ros_bridge_ws')

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
            RosWSBridge, timeout_sec=0.01
        ),
        1
    )
    # try:

    # except KeyboardInterrupt:
    #     log.warning('Terminating WS')

    shutdown_ioloop(server)

    try:
        log.info(
            f"Websocket started: 'ws://{url['address']}:{url['port']}'")
        ros_node.start()
        start_ioloop()
    except KeyboardInterrupt:
        log.warning('Terminating WS')

    RosWSBridge.destroy_node()
    rclpy.shutdown()
    ros_node.stop()
