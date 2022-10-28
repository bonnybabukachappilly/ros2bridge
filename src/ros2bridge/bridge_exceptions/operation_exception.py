"""Operation Exception.

This file contains custom exception for ROS Operations.
    * Publisher
    * Subscriber
    * Service Client
    * Action Client


Class:
    OperationNotFoundException:
        Client requested operation cannot be found.
"""


class OperationNotFoundException(Exception):
    """Client requested operation cannot be found."""

    ...
