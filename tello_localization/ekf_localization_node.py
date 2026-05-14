import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

import tf2_ros
import numpy as np

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

        # 解決角度 wrapping (例如 359度 - 1度 應該是 -2度 而不是 358度)
        y[3,0] = (y[3,0] + np.pi) % (2 * np.pi) - np.pi
        y[4,0] = (y[4,0] + np.pi) % (2 * np.pi) - np.pi
        y[5,0] = (y[5,0] + np.pi) % (2 * np.pi) - np.pi

        S = H @ self.Sigma @ H.T + self.Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ y
        I = np.eye(6)
        self.Sigma = (I - K @ H) @ self.Sigma

    def predict_timer(self):

        # fake control input
        #u = np.array([0.1, 0.0, 0.01, 0.0]).reshape(4,1)
        u = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)
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
        
        r = R.from_quat(quat)
        euler = r.as_euler('xyz') # roll, pitch, yaw
        z = np.array([
            [pose.position.x],
            [pose.position.y],
            [pose.position.z],
            [euler[0]], # roll
            [euler[2]], # yaw (根據你的定義是在 index 4)
            [euler[1]]  # pitch (根據你的定義是在 index 5)
        ])
        self.update(z)

    def publish_pose(self):
        now = self.get_clock().now().to_msg()

        # 建立 PoseStamped 訊息
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = now
        
        msg.pose.position.x = float(self.mu[0,0])
        msg.pose.position.y = float(self.mu[1,0])
        msg.pose.position.z = float(self.mu[2,0])
        # 將尤拉角轉回四元數發布
        # 再次提醒：注意你的 index 4 是 yaw, 5 是 pitch
        q = R.from_euler('xyz', [self.mu[3,0], self.mu[5,0], self.mu[4,0]]).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pose_pub.publish(msg)
        
        # Publish Path
        self.path_msg.poses.append(msg)
        self.path_pub.publish(self.path_msg)

        # Broadcast TF (map -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.mu[0,0])
        t.transform.translation.y = float(self.mu[1,0])
        t.transform.translation.z = float(self.mu[2,0])
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()