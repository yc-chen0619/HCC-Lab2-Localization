from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'tello_localization'
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'config.rviz')
    return LaunchDescription([
          # ====================================================
          # AprilTag Detector
          # ====================================================
          Node(package=package_name,
               executable='apriltag_detector_node',
               name='apriltag_detector_node',
               output='screen'),

          # ====================================================
          # EKF Localization
          # ====================================================
          Node(package=package_name,
               executable='ekf_localization_node',
               name='ekf_localization_node',
               output='screen'),

          # ====================================================
          # Static Tag TF
          # ====================================================
          Node(package=package_name,
               executable='tag_tf_broadcaster',
               name='tag_tf_broadcaster',
               output='screen'),

          # ====================================================
          # AprilTag Marker Visualization
          # ====================================================
          Node(package=package_name,
               executable='apriltag_visualizer',
               name='apriltag_visualizer',
               output='screen'),

          # ====================================================
          # RViz
          # ====================================================
          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               output='screen',
               arguments=['-d', rviz_config])
     ])

# ====================================================
# AprilTag Marker Visualization
# ====================================================
'''
Node(package=package_name,
     executable='apriltag_visualizer',
     name='apriltag_visualizer',
     output='screen'),
'''