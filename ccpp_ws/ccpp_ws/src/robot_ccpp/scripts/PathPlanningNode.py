import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from nav2_costmap_2d.costmap_2d_ros import Costmap2DROS

class CleaningPathPlanner(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.get_logger().info('Cleaning Path Planner Node started')

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.costmap = Costmap2DROS('cleaning_costmap', self.tf)

        # Wait for the costmap to initialize
        rclpy.spin_once(self, timeout_sec=5.0)

    def get_path_in_ros(self):
        # Your path planning logic here
        pass

    def publish_coverage_path(self):
        # Your path publishing logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CleaningPathPlanner()
    node.get_path_in_ros()

    r = rclpy.create_rate(1)  # 1 Hz
    while rclpy.ok():
        node.publish_coverage_path()
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
