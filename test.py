from mcp_server import locate_bot
from utils import msg2dict
from sensor_msgs.msg import Image


pose = locate_bot()
print(pose)
