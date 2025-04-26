import os
import subprocess
from functools import wraps
from contextlib import contextmanager
from typing import Callable, Any, Tuple, Union, TypeVar, Generator

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.wait_for_message import wait_for_message


def run_command(command_fmt: str):
    """Decorator to run a command after a function returns a value.
    :param command_fmt: The command to run, with a placeholder for the function return value.
    """
    def decorator(func: Callable[[Any], None | str | Tuple[str, ...]]) -> Callable[[Any], None | str | Tuple[str, ...]]:
        @wraps(func)
        def decorated(*args, **kwargs) -> None | str | Tuple[str, ...]:    
            result = func(*args, **kwargs)
            result = result if isinstance(result, tuple) else (result,)
            result = map(lambda word: '' if word is None else word, result)
            command = command_fmt.format(*result)
            command = " ".join(map(lambda line: line.strip(), command.split('\n')))
            print(f'(subprocess) {os.getcwd()}> {command}')
            command_seq = command.split()
            subprocess.run(command_seq)
            return result
        return decorated
    return decorator


@contextmanager
def ros2_node(node_name: str, args: list = None) -> Generator[Node, None, None]:
    '''
    provides a context manager to create a ROS2 node and clean up afterward.
    :param node_name: name of the ROS2 node
    :param args: list of command line arguments to be passed to rclpy.init()
    :yields: a ROS2 node
    '''
    rclpy.init(args=args)
    node: Node = rclpy.create_node(node_name)
    yield node
    node.destroy_node()
    rclpy.shutdown()


# Type of ROS2 message
MsgType = TypeVar('MsgType')  
# Type of ROS2 message processed result
# e.g, sensor_msgs/Image will be processed to np.array,
# so corresponding result type will be np.ndarray, whatever U want, etc
ProcessedResultType = TypeVar('ProcessedMsgType')


@contextmanager
def ros2_msg_sub(
    msg_type: MsgType,
    topic: str,
    node_name: str = None,
    *,
    func: Callable[[MsgType], ProcessedResultType] = None,
    qos_profile: Union[QoSProfile, int] = 1,
    time_to_wait: float = -1,
    args: list = None
) -> Generator[Tuple[bool, ProcessedResultType], None, None]:
    '''
    provide a context manager to subscribe to a ROS2 topic and process the received message.
    details look at the function `rclpy.wait_for_message.wait_for_message`
    :param msg_type: ROS2 message type to subscribe to
    :param topic: ROS2 topic to subscribe to
    :param node_name: name of the ROS2 node to create, if None, a default name will be used
    :param func: function to process the received message, if None, the received message will be returned as is
    :param qos_profile: ROS2 QoSProfile to subscribe with, or an integer for QoSProfile.DEFAULT
    :param time_to_wait: time to wait for a message to arrive, in seconds
    :param args: list of command line arguments to be passed to rclpy.init()
    :yields: a tuple of (success: bool, processed_msg: ProcessedResultType)
    '''
    if node_name is None:
        node_name = f'sub_{topic.replace("/", "_")}_node'
    if func is None:
        func = lambda x: x
    with ros2_node(node_name, args=args) as node:
        success: bool
        msg: MsgType
        success, msg = wait_for_message(
            msg_type,
            node,
            topic,
            qos_profile=qos_profile,
            time_to_wait=time_to_wait,
        )
        processed_msg: ProcessedResultType = func(msg) if success else None
        yield success, processed_msg

