import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from cv_bridge import CvBridge

import cv2
import numpy as np
import yaml

from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


class AprilTagDetectorNode(Node):

    def __init__(self):
        super().__init__('apriltag_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseArray, '/apriltag/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/apriltag/markers', 10)

        self.detector = Detector(families='tag36h11', 
                                 nthreads=1, 
                                 quad_decimate=1.0, 
                                 quad_sigma=0.0,
                                 refine_edges=1,
                                 decode_sharpening=0.25,
                                 debug=0)
        self.get_logger().info("Image subscriber with pupil_apriltags initialized.")

        self.fx = 920.0
        self.fy = 920.0
        self.cx = 480.0
        self.cy = 360.0
        self.tag_size = 0.16
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

    def get_tag_world_pose(self, tag_id):
        """
        Returns a 4x4 transformation matrix T_tag_in_world from YAML.
        """
        tag = self.tag_pose_dict.get(tag_id)
        if tag is None:
            self.get_logger().warn(f"No world pose defined for tag ID {tag_id}")
            return None

        pos = tag['position']
        rpy = tag['orientation_rpy']
        rot = R.from_euler('xyz', rpy).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        return T
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray,
                                    estimate_tag_pose=True,
                                    camera_params=self.camera_params,
                                    tag_size=self.tag_size)

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for idx, tag in enumerate(tags):
            t = tag.pose_t.flatten()
            rot = tag.pose_R

            euler = R.from_matrix(rot).as_euler('xyz')

            pose = Pose()
            pose.position.x = float(t[0])
            pose.position.y = float(t[1])
            pose.position.z = float(t[2])

            q = R.from_matrix(rot).as_quat()

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pose_array.poses.append(pose)

            marker = Marker()

            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose = pose

            marker.scale.x = 0.16
            marker.scale.y = 0.16
            marker.scale.z = 0.01

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()