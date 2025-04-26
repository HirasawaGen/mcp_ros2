import json
import time
from math import asin

import cv2
import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message

from mcp.server.fastmcp import FastMCP
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

from utils import run_command, ros2_msg_sub


mcp = FastMCP('Ros2 MCP Server')


@run_command('ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {}')
def cmd_vel(linear_x: float, angular_z: float) -> str:
    msg = {
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
    return json.dumps(msg, separators=(',', ':'), indent=None)


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
    cmd_vel(linear_x, angular_z)
    time.sleep(seconds)
    cmd_vel(0, 0)


@mcp.tool()
def save_image(path: str) -> bool:
    """
    use cv2.imwrite() to write the image captured by the ros2 bot to the specified path.
    :param path: The path to save the image.
    :return: True if the image is saved successfully, False otherwise.
    """
    with ros2_msg_sub(
        Image,
        '/camera/image_raw',
        func = lambda msg: np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1),
        qos_profile=qos_profile_sensor_data,
        time_to_wait=10.0
    ) as (success, image):
        if not success: return False
        cv2.imwrite(path, image)
        return success
    

@mcp.tool()
def locate_bot():
    """
    get the current position of the ros2 bot.
    you will know where th bot is by call this tool.
    :return: a dictionary containing the linear_x, linear_y, and angular_z. or empty dict if failed.
    """
    with ros2_msg_sub(
        TFMessage,
        '/tf',
        func = lambda msg: {
            'linear_x': msg.transforms[0].transform.translation.x,
            'linear_y': msg.transforms[0].transform.translation.y,
            'angular_z': 2 * asin(msg.transforms[0].transform.rotation.z)
        },
        qos_profile=qos_profile_sensor_data,
        time_to_wait=10.0
    ) as (success, twist):
        if not success: return {}
        return twist


if __name__ == '__main__':
    mcp.run(transport='stdio')
    rclpy.shutdown()
