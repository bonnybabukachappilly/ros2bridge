"""Data Parser.

Module for handling types of messages, services and action.
This module can be run individually to find out the structure
of ros operation data structure. Edit details under if __name__

Class:
    RosDataType(Enum):
        Available type ROS interface.
    RosDataParser:
        Ros data parser handle msg, srv and action ros types.
"""
from array import array
from enum import Enum
from importlib import import_module
from sys import modules
from typing import Any, Dict

import numpy as np


class RosDataType(Enum):
    """Available type ROS interface."""

    MESSAGE = 'msg'
    SERVICE = 'srv'
    ACTION = 'action'


class RosDataParser:
    """
    Ros data parser handle msg, srv and action ros types.

    Attributes:
        _data_type: str
            Ros data type of given instance.

    Methods:
        import_type(self, package: str) -> Any:
            Imports client requested module.

        get_module_instance(self, module: Any) -> Any:
            Create instance of imported module based of types.

        get_struct(self, module: Any, out: Dict[Any, Any]) -> Dict[Any, Any]:
            Create a structure of ros type.

        pack_data_to_ros(self, data: Dict[Any, Any], module: Any) -> Any:
            Convert client message to ros data type.

        pack_data_to_json(
            self, module: Any, output: Dict[Any, Any]
        ) -> Dict[Any, Any]:
            Convert ros data types to json serializable data.
    """

    def __init__(self, data_type: RosDataType) -> None:
        """
        Ros data parser handle msg, srv and action ros types.

        Args:
            data_type (RosDataType): Data type of instance.

        Raises:
            ValueError: data_type must be an instance of RosDataType.
        """
        if not isinstance(data_type, RosDataType):
            raise ValueError
        self._data_type = data_type.value

    def import_type(self, package: str) -> Any:
        """
        Import client requested module.

        Args:
            package (str): Package requested by client.

        Returns:
            Any: Imported package object requested by client.
        """
        _type = package.split('/')

        parent = f'{_type[0]}.{self._data_type}'
        child = _type[1]

        package = modules.get(  # type: ignore [assignment]
            parent) if parent in modules else import_module(parent)

        return getattr(package, child)

    def get_module_instance(self, module: Any) -> Any:
        """Create instance of imported module based of types.

        Args:
            module (Any): If given module name as str, import it.

        Returns:s
            Any: Instance of package requested by client.
        """
        if isinstance(module, str):
            module = self.import_type(module)

        if self._data_type == RosDataType.MESSAGE.value:
            return module()
        elif self._data_type == RosDataType.SERVICE.value:
            return module.Request()
        elif self._data_type == RosDataType.ACTION.value:
            return module.Goal()

    def get_struct(self, module: Any, out: Dict[Any, Any]) -> Dict[Any, Any]:
        """Create a structure of ros type.

        Args:
            module (Any): Module for which structure is generated.
            out (Dict[Any, Any]): out, Provide an empty Dict[Any, Any]: {}

        Returns:
            Dict[Any, Any]: Generated structure.
        """
        try:
            for slots in module.__slots__:
                out[slots] = {}
                out[slots] = self.get_struct(
                    getattr(module, slots), out[slots])

            return out

        except AttributeError:
            return type(module)  # type: ignore [return-value]

    def pack_data_to_ros(self, data: Dict[Any, Any], module: Any) -> Any:
        """
        Convert client message to ros data type.

        Args:
            data (Dict[Any, Any]): Data to pack as ros data.
            module (Any): Ros data to which user data is mapped.

        Returns:
            Any: Ros data.
        """
        # sourcery skip: remove-dict-keys
        try:
            for key in data.keys():
                value = self.pack_data_to_ros(data[key], getattr(module, key))
                setattr(module, key, value)

            return module
        except AttributeError:
            return data

    def pack_data_to_json(
            self, module: Any, output: Dict[Any, Any]) -> Dict[Any, Any]:
        """Convert ros data types to json serializable data.

        Args:
            module (Any): Ros data to be unpacked.
            output (Dict[Any, Any]): Outputs,
                Provide an empty Dict[Any, Any]: {}

        Returns:
            Dict[Any, Any]: Json serializable data.
        """
        # sourcery skip: assign-if-exp, reintroduce-else

        try:
            for slots in module.__slots__:
                slots = slots.strip('_')
                output[slots] = {}
                output[slots] = self.pack_data_to_json(
                    getattr(module, slots), output[slots])

            return output

        except AttributeError:
            if isinstance(module, (array, np.ndarray)):
                return module.tolist()  # type: ignore [return-value]
            return module  # type: ignore [no-any-return]


if __name__ == '__main__':
    _type = RosDataType.ACTION
    _action = 'nav2_msgs/FollowPath'

    ros_data = RosDataParser(data_type=_type)

    _struct = ros_data.get_struct(
        module=ros_data.get_module_instance(
            module=_action
        ),
        out={}
    )

    print(_struct)
