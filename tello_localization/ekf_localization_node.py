import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Path

import tf2_ros

import numpy as np
import yaml

from scipy.spatial.transform import Rotation as R


class EKFLocalizationNode(Node):

    def __init__(self):
        super().__init__('ekf_localization_node')
        self.subscription = self.create_subscription(PoseArray, '/apriltag/detections', self.detection_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        self.path_pub = self.create_publisher(Path, '/ekf_path', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.dt = 0.1
        self.mu = np.zeros((6,1))
        self.Sigma = np.eye(6) * 0.1
        # [x, y, z, roll, yaw, pitch]
        self.Rm = np.diag([0.05, 0.05, 0.05, 0.02, 0.02, 0.02])
        # [x, y, z, roll, yaw, pitch]
        self.Q = np.diag([0.03, 0.03, 0.03, 0.05, 0.05, 0.05])

        self.timer = self.create_timer(self.dt, self.predict_timer)

    # state : [x, y, z, roll, yaw, pitch]
    # control : [v, roll_rate, yaw_rate, pitch_rate]
    def motion_model(self, x, u):
        dt = self.dt

        px = x[0,0]
        py = x[1,0]
        pz = x[2,0]
        roll = x[3,0]
        yaw = x[4,0]
        pitch = x[5,0]

        v = u[0,0]
        roll_rate = u[1,0]
        yaw_rate = u[2,0]
        pitch_rate = u[3,0]

        x_pred = np.zeros((6,1))
        x_pred[0,0] = px + v*np.cos(yaw)*np.cos(pitch)*dt
        x_pred[1,0] = py + v*np.sin(yaw)*np.cos(pitch)*dt
        x_pred[2,0] = pz + v*np.sin(pitch)*dt
        x_pred[3,0] = roll + roll_rate*dt
        x_pred[4,0] = yaw + yaw_rate*dt
        x_pred[5,0] = pitch + pitch_rate*dt
        return x_pred

    def jacobian_F(self, x, u):
        dt = self.dt

        yaw = x[4,0]
        pitch = x[5,0]

        v = u[0,0]

        F = np.eye(6)
        F[0,4] = -v * np.sin(yaw) * np.cos(pitch) * dt
        F[0,5] = -v * np.cos(yaw) * np.sin(pitch) * dt
        F[1,4] =  v * np.cos(yaw) * np.cos(pitch) * dt
        F[1,5] = -v * np.sin(yaw) * np.sin(pitch) * dt
        F[2,5] =  v * np.cos(pitch) * dt
        return F
    
    def predict(self, u):
        self.mu = self.motion_model(self.mu, u)
        F = self.jacobian_F(self.mu, u)
        self.Sigma = F @ self.Sigma @ F.T + self.Rm

    def update(self, z):
        H = np.eye(6)
        z_pred = self.mu
        y = z - z_pred
        S = H @ self.Sigma @ H.T + self.Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ y
        I = np.eye(6)
        self.Sigma = (I - K @ H) @ self.Sigma

    def predict_timer(self):

        # fake control input
        u = np.array([0.1, 0.0, 0.01, 0.0]).reshape(4,1)
        self.predict(u)

        self.publish_pose()

    def detection_callback(self, msg):
        if len(msg.poses) == 0:
            return

        pose = msg.poses[0]
        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]
        euler = R.from_quat(quat)