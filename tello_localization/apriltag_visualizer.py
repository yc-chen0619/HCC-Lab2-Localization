from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np

def create_robot_marker(node, mu):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = 'robot'
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose.position.x = float(mu[0])
    marker.pose.position.y = float(mu[1])
    marker.pose.position.z = float(mu[2])

    marker.scale.x = 0.3
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0

    return marker