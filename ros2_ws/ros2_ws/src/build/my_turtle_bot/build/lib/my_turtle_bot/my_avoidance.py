import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Use simulation time
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Parameters for obstacle avoidance behavior
        self.fast_turn = 1.01 # Angular velocity for fast turn
        self.left_turn = 0.51  # Angular velocity for left turn
        self.right_turn = -0.58  # Angular velocity for right turn

        # Scoring logic
        self.scores = 0

        # Publisher to the velocity topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscription to the LIDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        # Subscription to the Clock topic
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        self.get_logger().debug('Obstacle Avoidance Node has been started.')
        self.twist = Twist()
        self.reset_triggered = False  # Flag to track reset state

        # Start time of the simulation
        self.start_time = None
        self.simulation_duration = 60.0  # Duration for each training period (in seconds)

    def lidar_callback(self, msg):
        # Get distances for front-left, front-right, and directly in front
        left_dist = min(msg.ranges[0:20])
        right_dist = min(msg.ranges[340:359])
        front_dist = min(left_dist, right_dist)

        # Define distance thresholds
        threshold = 0.5  # Distance to trigger avoidance
        close_threshold = 0.15  # Close proximity

        # Decision-making and scoring logic
        if right_dist < close_threshold or left_dist < close_threshold:
            self.twist.linear.x = -1.5  # Move backward
            self.twist.angular.z = 0.0  # Stop rotation
            self.scores += -10
            self.get_logger().debug(
                f'Hit the obstacle! Distance to obstacle: {front_dist:.2f} meters. '
                f'Angular velocity: {self.twist.angular.z} rad/s. Current score: {self.scores}')
        elif right_dist < threshold and left_dist < threshold:
            self.twist.linear.x = 0.0  # Stop forward movement
            self.twist.angular.z = self.fast_turn  # Fast turn
            self.scores += -1
            self.get_logger().debug(
                f'FRONT obstacle detected! Distance: {front_dist:.2f} meters. '
                f'Angular velocity: {self.twist.angular.z} rad/s. Current score: {self.scores}')
        elif right_dist < threshold:
            self.twist.linear.x = 0.0  # Stop forward movement
            self.twist.angular.z = self.left_turn  # Turn left
            self.scores += -1
            self.get_logger().debug(
                f'RIGHT obstacle detected! Distance: {right_dist:.2f} meters. '
                f'Angular velocity: {self.twist.angular.z} rad/s. Current score: {self.scores}')
        elif left_dist < threshold:
            self.twist.linear.x = 0.0  # Stop forward movement
            self.twist.angular.z = self.right_turn  # Turn right
            self.scores += -1
            self.get_logger().debug(
                f'LEFT obstacle detected! Distance: {left_dist:.2f} meters. '
                f'Angular velocity: {self.twist.angular.z} rad/s. Current score: {self.scores}')
        else:
            self.twist.linear.x = 0.5  # Move forward
            self.twist.angular.z = 0.0  # No rotation
            self.scores += 3
            self.get_logger().debug(
                f'Path clear. Moving forward. Front distance: {front_dist:.2f} meters. Current score: {self.scores}')

        # Log the current velocity commands
        self.get_logger().debug(
            f'Linear velocity: {self.twist.linear.x} m/s, Angular velocity: {self.twist.angular.z} rad/s')

        # Publish the velocity command
        self.publisher_.publish(self.twist)

    def clock_callback(self, msg):
        # Get the current simulation time in seconds
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.start_time is None:
            # Initialize the start time
            self.start_time = current_sim_time

        # Check if the specified duration has passed
        if current_sim_time - self.start_time >= self.simulation_duration:
            self.get_logger().debug(
                f"Time for this training period ({self.simulation_duration} seconds) has elapsed.")
            self.reset_simulation()
            self.start_time = current_sim_time  # Reset start time for the next period

    def reset_simulation(self):
        self.get_logger().debug("Resetting simulation...")
        client = self.create_client(Empty, '/reset_simulation')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Waiting for reset_simulation service...')

        request = Empty.Request()
        future = client.call_async(request)
        self.get_logger().info(f'Time up! Final Score: {self.scores}')
        print(f'  Fast Turn Speed: {self.fast_turn} rad/s')
        print(f'  Left Turn Speed: {self.left_turn} rad/s')
        print(f'  Right Turn Speed: {self.right_turn} rad/s')
        self.reset_triggered = True


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if node.reset_triggered:  # Break the spin loop if reset is triggered
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
