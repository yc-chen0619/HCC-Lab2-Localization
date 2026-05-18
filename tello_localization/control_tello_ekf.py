#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
-----------------------------------------
Tello & EKF 鍵盤控制節點 (GTA 5 飛行設置)
-----------------------------------------
左手區 (WASD) - 基礎起降與水平轉向：
    w / s : 上升 / 下降 (v_z / linear.z)
    a / d : 左轉 / 右轉 (yaw_rate / angular.z)

右手區 (右側數字鍵 8456) - 姿態與推力推進：
    8 / 5 : 前進 / 後退 (v_x / linear.x) [連動 pitch_rate 壓/拉機頭]
    4 / 6 : 左翻滾 / 右翻滾 (roll_rate / angular.x)

空白鍵 (Space) : 全部速度與姿態歸零 (緊急煞車懸停)
CTRL+C : 退出
-----------------------------------------
提示：請確保鍵盤的 Num Lock 已開啟！
-----------------------------------------
"""

# 設定鍵盤控制單次變動步長
SPEED_STEP = 0.2
ANGLE_STEP = 0.2

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

class TeleopTelloEKF(Node):
    def __init__(self):
        super().__init__('teleop_tello_ekf')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # initial control signal
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = float(self.v_x)
        twist.linear.y = float(self.v_y)
        twist.linear.z = float(self.v_z)
        twist.angular.x = float(self.roll_rate)
        twist.angular.y = float(self.pitch_rate)
        twist.angular.z = float(self.yaw_rate)
        self.publisher_.publish(twist)
        
        # 顯示當前發出的控制向量 u
        sys.stdout.write(f"\rinput control = [vx: {self.v_x:.2f}, vy: {self.v_y:.2f}, vz: {self.v_z:.2f}, " \
                          "roll_rate: {self.roll_rate:.2f}, pitch_rate: {self.pitch_rate:.2f}, yaw_rate: {self.yaw_rate:.2f}]")
        sys.stdout.flush()

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopTelloEKF()

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)
            # --- Special function keys ---
            if key == ' ':            # stop keys
                node.v_x = 0.0
                node.v_y = 0.0
                node.v_z = 0.0
                node.yaw_rate = 0.0
                node.roll_rate = 0.0
                node.pitch_rate = 0.0
            if key == '\x03':         # CTRL+C
                break

            # --- Left hand = WASD ---
            if key == 'w':
                node.v_z += SPEED_STEP
            elif key == 's':
                node.v_z -= SPEED_STEP
            elif key == 'a':
                node.yaw_rate += ANGLE_STEP
            elif key == 'd':
                node.yaw_rate -= ANGLE_STEP
            # --- Right hand = 8456 ---
            elif key == '8':
                node.v_x += SPEED_STEP
                node.pitch_rate += ANGLE_STEP
            elif key == '5':
                node.v_x -= SPEED_STEP
                node.pitch_rate -= ANGLE_STEP
            elif key == '4':
                node.v_y += SPEED_STEP
                node.roll_rate += ANGLE_STEP
            elif key == '6':
                node.v_y -= SPEED_STEP
                node.roll_rate -= ANGLE_STEP
            
            node.publish_cmd()
            rclpy.spin_once(node, timeout_sec=0.05)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()