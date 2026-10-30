import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AvoidNode(Node):
    """TurtleBot3 激光雷达避障节点。"""

    def __init__(self):
        super().__init__('avoid_node')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.safe_distance = 0.4    # 安全距离 (m)
        self.linear_speed = 0.2     # 直行线速度 (m/s)
        self.angular_speed = 0.6    # 右转角速度 (rad/s)
        self.front_angle = 0.35     # 正前方半扇区角 (rad)，约 20°

    def scan_callback(self, msg):
        # 取正前方扇区，过滤 inf/nan 无效值
        half = int(self.front_angle / msg.angle_increment)
        n = len(msg.ranges)
        center = n // 2
        front = msg.ranges[center - half:center + half + 1]
        valid = [r for r in front if math.isfinite(r)]
        if not valid:
            return

        twist = Twist()
        if min(valid) < self.safe_distance:
            # 离障碍物近：停止前进，原地右转
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            # 距离足够：直线前进
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
