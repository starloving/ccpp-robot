import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import cv2
import numpy as np

class CleaningPathPlanning(Node):
    def __init__(self, costmap2d_ros):
        super().__init__('cleaning_path_planning')
        self.costmap2d_ros = costmap2d_ros
        self.costmap2d = self.costmap2d_ros.get_costmap()

        self.plan_pub = self.create_publisher(Path, 'cleaning_path', 1)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'covered_grid', 1)

        size_of_cell = 3
        grid_covered_value = 0

        if self.get_parameter('size_of_cell').get_parameter_value().type == rclpy.Parameter.Type.NOT_SET:
            self.set_parameters([rclpy.parameter.Parameter('size_of_cell', rclpy.Parameter.Type.INTEGER, size_of_cell)])

        if self.get_parameter('grid_covered_value').get_parameter_value().type == rclpy.Parameter.Type.NOT_SET:
            self.set_parameters([rclpy.parameter.Parameter('grid_covered_value', rclpy.Parameter.Type.INTEGER, grid_covered_value)])

        sizex = self.costmap2d.get_size_in_cells_x()
        sizey = self.costmap2d.get_size_in_cells_y()
        resolution = self.costmap2d.get_resolution()
        src_map = np.zeros((sizey, sizex), dtype=np.uint8)

        for r in range(sizey):
            for c in range(sizex):
                src_map[r, c] = self.costmap2d.get_cost(c, sizey - r - 1)

        self.initialize_mats()
        self.initialize_covered_grid()

    def initialize_mats(self):
        pass

    def initialize_covered_grid(self):
        pass

    def get_path_in_ros(self):
        pass

    def get_border_tracking_path_in_ros(self):
        pass

    def get_border_tracking_path_in_cv(self, result_vec):
        pass

    def set_covered_grid(self, wx, wy):
        pass

    def publish_grid(self):
        pass

    def get_path_in_cv(self):
        pass

    def publish_coverage_path(self):
        pass

    def publish_plan(self, path):
        pass

    def cell_contains_point(self, pt, cell):
        pass

    def main_planning_loop(self):
        pass

    def distance(self, pta, ptb):
        pass

    def find_element(self, points_vec, pt):
        pass

    def initialize_covered_grid(self):
        pass

    def bounding_judge(self, a, b):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CleaningPathPlanning(None)  # You need to pass the costmap2d_ros instance here
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
