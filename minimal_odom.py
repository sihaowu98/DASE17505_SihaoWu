#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import time

class MinimalOdom(Node):
    def __init__(self):
        super().__init__('minimal_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        print("=== 最小odom发布者启动 ===")
        print("正在发布 /odom 话题数据...")
        
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        # 模拟机器人在原点附近缓慢移动
        self.theta += 0.05  # 缓慢旋转
        self.x = 0.1 * time.time() % 1.0  # 在x方向缓慢移动
        self.y = 0.0  # 保持在y=0
        
        msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MinimalOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n=== 停止odom发布 ===")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()