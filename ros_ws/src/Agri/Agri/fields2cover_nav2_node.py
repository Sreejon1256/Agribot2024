# Import necessary modules from fields2cover
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from fields2cover.planning import BoustrophedonPathPlanner
from fields2cover.geometry import Polygon

class CoveragePathPublisher(Node):
    def __init__(self):
        super().__init__('fields2cover_coverage_publisher')
        # Create publisher to publish coverage path
        self.path_publisher = self.create_publisher(Path, 'coverage_path', 10)

        # Define the field (example: simple square)
        self.field = Polygon()
        self.field.addPoint(0.0, 0.0)
        self.field.addPoint(10.0, 0.0)
        self.field.addPoint(10.0, 10.0)
        self.field.addPoint(0.0, 10.0)
        self.field.addPoint(0.0, 0.0)

        # Width of the robot (for coverage)
        self.robot_width = 0.5  # Adjust this value to your robot's width

        # Generate and publish coverage path
        self.generate_and_publish_path()

    def generate_and_publish_path(self):
        # Create a coverage path planner (Boustrophedon pattern)
        planner = BoustrophedonPathPlanner()

        # Generate the coverage path for the defined field
        coverage_paths = planner.generateCoveragePaths(self.field, self.robot_width)

        # Publish the coverage paths
        self.publish_coverage_paths(coverage_paths)

    def publish_coverage_paths(self, coverage_paths):
        # Prepare the Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Frame for RViz visualization

        # Convert Fields2Cover paths into ROS 2 PoseStamped messages
        for path in coverage_paths:
            for point in path:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = point.x()
                pose_stamped.pose.position.y = point.y()
                pose_stamped.pose.orientation.w = 1.0  # No orientation applied for now
                path_msg.poses.append(pose_stamped)

        # Publish the Path message
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published coverage path.")

def main(args=None):
    rclpy.init(args=args)
    coverage_path_publisher = CoveragePathPublisher()

    try:
        rclpy.spin(coverage_path_publisher)
    except KeyboardInterrupt:
        coverage_path_publisher.get_logger().info("Shutting down.")
    finally:
        coverage_path_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
