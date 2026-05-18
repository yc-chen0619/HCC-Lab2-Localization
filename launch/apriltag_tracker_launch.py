from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
     package_name = 'tello_localization'
     rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'config.rviz')

     # ====================================================
     # AprilTag Detector
     # ====================================================
     apriltag_node = Node(package=package_name,
                          executable='apriltag_detector_node',
                          name='apriltag_detector_node',
                          output='screen')

     # ====================================================
     # Static Tag TF
     # ====================================================
     tag_tf_node = Node(package=package_name,
                        executable='tag_tf_broadcaster',
                        name='tag_tf_broadcaster',
                        output='screen')
     
     # ====================================================
     # EKF Localization
     # ====================================================
     ekf_node = Node(package=package_name,
                    executable='ekf_localization_node',
                    name='ekf_localization_node',
                    output='screen')

     # ====================================================
     # Tello Control & EKF input (using prefix to open new terminal)
     # ====================================================
     tello_node = Node(package='tello_ros',
                       executable='tello_node',
                       name='tello_node',
                       output='screen',
                       remappings=[('/tello/cmd_vel', '/cmd_vel')])  # 關鍵：把 Tello 預設的 /tello/cmd_vel 對齊到全域的 /cmd_vel

     control_node = Node(package=package_name,
                         executable='control_tello_ekf',
                         name='control_tello_ekf',
                         output='screen',
                         prefix=['gnome-terminal -- '])

     # ====================================================
     # 把 camera_frame 綁定在 base_link 上面
     # ====================================================
     static_tf_node = Node(package='tf2_ros',
                           executable='static_transform_publisher',
                           name='camera_base_link_tf',
                           arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'camera_frame'])

     # ====================================================
     # RViz
     # ====================================================
     rviz_node = Node(package='rviz2',
                      executable='rviz2',
                      name='rviz2',
                      output='screen',
                      arguments=['-d', rviz_config])
     
     return LaunchDescription([ekf_node, tello_node, control_node, apriltag_node, tag_tf_node, static_tf_node, rviz_node])

# ====================================================
# AprilTag Marker Visualization
# ====================================================
'''
Node(package=package_name,
     executable='apriltag_visualizer',
     name='apriltag_visualizer',
     output='screen'),
'''