import os
import subprocess
from functools import wraps
from contextlib import contextmanager
import time
from typing import (
    Callable,
    Any,
    Tuple,
    Union,
    Type,
    TypeVar,
    Generator,
    Sequence
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.wait_for_message import wait_for_message

from rosidl_parser.definition import BASIC_TYPES


def run_command(command_fmt: str):
    """
    Decorator to run a command after a function returns a value.
    useless temporarily
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
def ros2_context(args: list = None) -> Generator[None, None, None]:
    rclpy.init(args=args)
    yield
    rclpy.shutdown()
        

@contextmanager
def ros2_node(node_name: str, args: list = None) -> Generator[Node, None, None]:
    with ros2_context(args):
        node = rclpy.create_node(node_name)
        yield node
        node.destroy_node()


# Type of ROS2 message
MsgType = TypeVar('MsgType')
# Type of ROS2 message processed result
# e.g, sensor_msgs/Image will be processed to np.array,
# so corresponding result type will be np.ndarray, whatever U want, etc
ProcessedResultType = TypeVar('ProcessedMsgType')


def dict2msg(msg_dict: dict, msg_type: Type[MsgType]) -> MsgType:
    '''
    convert a dictionary to a ROS2 message use recursion.
    :param msg_dict: dictionary to convert
    :param msg_type: ROS2 message type to convert to
    :returns: ROS2 message
    '''
    msg = msg_type()
    for key, value in msg_dict.items():
        setted_value = None
        if isinstance(value, dict):
            setted_value = dict2msg(value, type(getattr(msg, key)))
        elif isinstance(value, list):
            setted_value = [dict2msg(item, type(getattr(msg, key)[0])) for item in value]
        else:
            setted_value = value
        if setted_value is not None:
            setattr(msg, key, setted_value)
    return msg


def msg2dict(msg: MsgType) -> dict:
    '''
    convert a ROS2 message to a dictionary use recursion.
    :param msg: ROS2 message to convert
    :returns: dictionary
    '''
    msg_dict = {}
    for field, field_type in msg.get_fields_and_field_types().items():
        value = getattr(msg, field)
        if field_type in BASIC_TYPES:
            msg_dict[field] = getattr(msg, field)
        elif field_type.startswith('sequence<'):
            begin_idx = field_type.find('<')
            end_idx = field_type.find('>', begin_idx+1)
            item_type = field_type[begin_idx+1:end_idx]
            if item_type in BASIC_TYPES:
                msg_dict[field] = value
            else:
                msg_dict[field] = [msg2dict(item) for item in value]
        elif isinstance(value, str):
            msg_dict[field] = value
        else:
            msg_dict[field] = msg2dict(value)
    return msg_dict


def ros2_msg_sub(
    msg_type: Type[MsgType],
    topic: str,
    node_name: str = None,
    count: int = 1,
    *,
    func: Callable[[MsgType], ProcessedResultType] = None,
    qos_profile: Union[QoSProfile, int] = 1,
    time_to_wait: float = -1.0,
    args: list = None
) -> Generator[Tuple[ProcessedResultType | None, ...] | ProcessedResultType | None, None, None]:
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
        node_name = f'sub{topic.replace("/", "_")}_node'
    if func is None:
        func = lambda msg: msg2dict(msg)
    with ros2_node(args=args, node_name=node_name) as node:
        msgs = [0 for _ in range(count)]
        for i in range(count):
            success: bool
            msg: MsgType
            success, msg = wait_for_message(
                msg_type,
                node,
                topic,
                qos_profile=qos_profile,
                time_to_wait=time_to_wait,
            )
            msgs[i] = func(msg) if success else None
    return msgs[0] if count == 1 else tuple(msgs)


def ros2_msg_pub(
    msg_type: Type[MsgType],
    topic: str,
    msg_dicts: Sequence[dict] | dict,
    node_name: str = None,
    *,
    qos_profile: Union[QoSProfile, int] = 1,
    time_to_wait: float = -1.0,
    args: list = None
) -> None:
    '''
    publish one or more ROS2 messages to a topic.
    :param msg_type: ROS2 message type to publish
    :param topic: ROS2 topic to publish to
    :param msg_dicts: a sequence of dictionaries or a single dictionary to be converted to ROS2 messages and published
    :param node_name: name of the ROS2 node to create, if None, a default name will be used
    :param qos_profile: ROS2 QoSProfile to publish with, or an integer for QoSProfile.DEFAULT
    :param args: list of command line arguments to be passed to rclpy.init()
    :returns: None
    '''
    if time_to_wait > 0.1 + 1e-6:
        time_to_wait -= 0.1
    if isinstance(msg_dicts, dict):
        msg_dicts = [msg_dicts]
    count = len(msg_dicts)
    msgs = map(lambda msg_dict: dict2msg(msg_dict, msg_type), msg_dicts)
    if node_name is None:
        node_name = f'pub{topic.replace("/", "_")}_node'
    with ros2_node(args=args, node_name=node_name) as node:
        publisher = node.create_publisher(msg_type, topic, qos_profile=qos_profile)
        for i, msg in enumerate(msgs):
            publisher.publish(msg)
            time.sleep(0.1 if i == (count - 1) or time_to_wait == -1.0 else time_to_wait)
        node.destroy_publisher(publisher)

