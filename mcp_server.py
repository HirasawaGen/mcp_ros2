from math import asin
from typing import List, Dict

import cv2
import numpy as np

from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message

from mcp.server.fastmcp import FastMCP

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

from utils import ros2_msg_pub, ros2_msg_sub


mcp = FastMCP('Ros2 MCP Server')




@mcp.tool()
def move(linear_x: float, angular_z: float, seconds: float) -> None:
    """
    move the ros2 bot with the given linear and angular velocities for the specified number of seconds.
    If we need the ros2 bot move to it's straight front, argument linear_x should be positive, argument angular_z should be zero.
    If we need the ros2 bot move to it's left front, argument linear_x and angular_z should be both positive.
    If we need the ros2 bot rotate, argument linear_x should be zero, argument angular_z should be not zero.
    And we can also set the duration of the action by the seconds argument.
    If we need the ros2 bot move a specific distance, we can use formula: distance = linear_x * seconds.
    For example, if we want the ros2 bot to move 1 meters, we can use vel_pub(0.2, 0, 5) or vel_pub(0.5, 0, 2). 
    
    :param linear_x: the linear velocity in x direction.
    :param angular_z: the angular velocity in z direction.
    :param seconds: the duration of the action.
    :return: None This function will return Nothing whatever success or not.
    """
    pose2twist = lambda linear_x, angular_z: {
        'linear': {
            'x': float(linear_x),
            'y': 0.0,
            'z': 0.0
        },
        'angular': {
            'x': 0.0,
            'y': 0.0,
            'z': float(angular_z)
        }
    }
    ros2_msg_pub(Twist, '/cmd_vel', [
        pose2twist(linear_x, angular_z),
        pose2twist(0, 0)
    ], time_to_wait=seconds)


@mcp.tool()
def save_image(path: str) -> bool:
    """
    use cv2.imwrite() to write the image captured by the ros2 bot to the specified path.
    :param path: The path to save the image.
    :return: True if the image is saved successfully, False otherwise.
    """
    image: np.ndarray | None = ros2_msg_sub(
        Image,
        '/camera/image_raw',
        func = lambda msg: np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1),
        qos_profile=qos_profile_sensor_data,
        time_to_wait=10.0
    )
    if image is None: return False
    cv2.imwrite(path, image)
    return True
    

@mcp.tool()
def locate_bot() -> Dict[str, float]:
    """
    get the current position of the ros2 bot.
    you will know where th bot is by call this tool.
    :return: a dictionary containing the linear_x, linear_y, and angular_z. or empty dict if failed.
    """
    pose: Dict[str, float] | None = ros2_msg_sub(
        TFMessage,
        '/tf',
        func = lambda msg: {
            'linear_x': msg.transforms[0].transform.translation.x,
            'linear_y': msg.transforms[0].transform.translation.y,
            'angular_z': 2 * asin(msg.transforms[0].transform.rotation.z)
        },
        qos_profile=qos_profile_sensor_data,
        time_to_wait=10.0
    )
    if pose is None: return {}
    return pose


@mcp.tool()
def collision_detect(threshold: float) -> tuple[str, ...]:
    """
    detect the collision between the ros2 bot and the environment.
    :param threshold: the threshold of the collision detection.
    :return: a tuple of strings containing the name of directions where the collision is detected.
    the return value will only contain 8 directions:
        'straight front', 'right front', 'straight right', 'right back'
        'straight back', 'left back', 'left front', 'straight left'
    """
    threshold = float(threshold)
    directions = (
        'straight front', 'right front', 'straight right', 'right back',
        'straight back', 'left back', 'straight left', 'left front'
    )
    ranges: np.ndarray | None = ros2_msg_sub(
        LaserScan,
        '/scan',
        func=lambda msg: msg.ranges,
        qos_profile=qos_profile_sensor_data,
        time_to_wait=10.0
    )
    if ranges is None: return []
    ranges = np.array([ranges[i] for i in range(0, 360, 360//8)])
    return tuple(filter(lambda d: ranges[directions.index(d)] < threshold, directions))


if __name__ == '__main__':
    mcp.run(transport='stdio')
