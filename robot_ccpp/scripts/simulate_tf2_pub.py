#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomToMapTFPublisher(Node):

    def __init__(self):
        super().__init__('odom_to_map_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_tf)  # 设置定时器以发布TF变换
        self.get_logger().info('odom to map tf publisher ...')

    def publish_tf(self):
        # 创建一个TF变换消息
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'  # 目标坐标系为'map'
        transform.child_frame_id = 'base_link'  # 源坐标系为'odom'
        # 设置变换矩阵
        transform.transform.translation.x = -1.68     # 以示例方式设置平移
        transform.transform.translation.y = -2.37
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0        # 以示例方式设置旋转

        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
