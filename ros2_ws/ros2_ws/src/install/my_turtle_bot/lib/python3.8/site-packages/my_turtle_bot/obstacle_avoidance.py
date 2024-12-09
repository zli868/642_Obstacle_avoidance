import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Publisher to the velocity topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscription to the LIDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.get_logger().info('Obstacle Avoidance Node has been started.')
        self.twist = Twist()

    def lidar_callback(self, msg):
        # Get the minimum distance in the front (ranges[0] is directly in front)
        front_distance = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]))

        # Define a threshold distance to trigger obstacle avoidance
        threshold = 0.4  # 0.4 meters

        # If there is an obstacle in front
        if front_distance < threshold:
            self.twist.linear.x = 0.0  # Stop moving forward
            self.twist.angular.z = 0.5  # Rotate to avoid the obstacle
            self.get_logger().info(f'Obstacle detected! Front distance: {front_distance:.2f} meters. Turning...')
        else:
            self.twist.linear.x = 0.5  # Move forward
            self.twist.angular.z = 0.0  # No rotation
            self.get_logger().info(f'Path clear. Moving forward. Front distance: {front_distance:.2f} meters.')

        # Log the current velocity commands (linear and angular)
        self.get_logger().info(
            f'Linear velocity: {self.twist.linear.x} m/s, Angular velocity: {self.twist.angular.z} rad/s')
        # Publish the velocity command
        self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

