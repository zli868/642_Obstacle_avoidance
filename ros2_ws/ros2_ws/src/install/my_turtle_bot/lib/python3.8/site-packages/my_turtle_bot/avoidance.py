import numpy as np
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock


class EvolutionaryObstacleAvoidance(Node):
    def __init__(self, individual):
        super().__init__('evolutionary_obstacle_avoidance')

        # Use simulation time
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Publisher to the velocity topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscription to the LIDAR scan topic
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # Subscription to the IMU sensor topic
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # Subscription to the Clock topic
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # Set timer for 40 seconds
        # self.timer = self.create_timer(40.0, self.reset_simulation)
        self.individual = individual  # a mapping
        self.scores = 0
        fast_turn = self.individual['fast_turn']
        left_turn = self.individual['left_turn']
        right_turn = self.individual['right_turn']
        self.get_logger().debug('Obstacle Avoidance Node has been started.'
                                f' fast turn speed {fast_turn} rad/s.'
                                f' left turn speed {left_turn} rad/s.'
                                f' right turn speed {right_turn} rad/s.')
        self.twist = Twist()
        self.distance_data = []
        # Variable to monitor the robot's orientation
        self.excessive_tilt_detected = False
        # Initialization code...
        self.reset_triggered = False  # Flag to track reset state

        # Start time of the simulation
        self.start_time = None
        self.simulation_duration = 60.0  # Duration for each training period (in seconds)

    def clock_callback(self, msg):
        # Get the current simulation time in seconds
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.start_time is None:
            # Initialize the start time
            self.start_time = current_sim_time

        # Check if the specified duration has passed
        if current_sim_time - self.start_time >= self.simulation_duration:
            self.get_logger().debug(f"Time for this training period ({self.simulation_duration} seconds) has elapsed.")
            self.reset_simulation()
            self.start_time = current_sim_time  # Reset start time for the next period

    def imu_callback(self, msg):
        # Monitor the roll and pitch angles to detect excessive tilt
        orientation_q = msg.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Define tilt threshold (in radians) beyond which the robot is considered to be unstable
        tilt_threshold = 0.5  # Approximately 28.65 degrees
        if abs(roll) > tilt_threshold or abs(pitch) > tilt_threshold:
            self.excessive_tilt_detected = True
            self.get_logger().warn(f'Excessive tilt detected! Roll: {roll:.2f}, Pitch: {pitch:.2f}, resetting')
            self.crash_reset_simulation()
        else:
            self.excessive_tilt_detected = False

    def quaternion_to_euler(self, x, y, z, w):
        # Function to convert quaternion to Euler angles (roll, pitch, yaw)
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def lidar_callback(self, msg):
        # Get the front-left and front-right distance in the front (ranges[0] is directly in front)
        left_dist = min(msg.ranges[0:20])
        right_dist = min(msg.ranges[340:359])
        front_dist = min(left_dist, right_dist)

        # Define a threshold distance to trigger obstacle avoidance
        threshold = 0.5  # 0.5 meters
        close_threshold = 0.15  # 0.5 meters
        # define our policy here. If left obstacle detected, turn right
        # if right detected, turn left
        # if both detected, turn left at a faster speed
        # if no obstacle detected, moving forward
        if right_dist < close_threshold or left_dist < close_threshold:
            self.twist.linear.x = -1.5  # moving back
            self.twist.angular.z = 0.0  # Rotate to avoid the obstacle
            self.scores += -10
            self.get_logger().debug(
                f'Hit the obstacle! distance to obstacle: {front_dist:.2f} meters. Turning at Angular velocity: {self.twist.angular.z} rad/s. current score:{self.scores}')
        elif right_dist < threshold and left_dist < threshold:
            self.twist.linear.x = 0.0  # Stop moving forward
            self.twist.angular.z = self.individual['fast_turn']  # Rotate to avoid the obstacle
            self.scores += -1
            self.get_logger().debug(
                f'FRONT Obstacle detected! distance to obstacle: {front_dist:.2f} meters. Turning at Angular velocity: {self.twist.angular.z} rad/s. current score:{self.scores}')
        elif right_dist < threshold:
            self.twist.linear.x = 0.0  # Stop moving forward
            self.twist.angular.z = self.individual['left_turn']  # Rotate to avoid the obstacle
            self.scores += -1
            self.get_logger().debug(
                f'RIGHT Obstacle detected! distance to obstacle: {right_dist:.2f} meters. Turning at Angular velocity: {self.twist.angular.z} rad/s. current score:{self.scores}')
        elif left_dist < threshold:
            self.twist.linear.x = 0.0  # Stop moving forward
            self.twist.angular.z = self.individual['right_turn']  # Rotate to avoid the obstacle
            self.scores += -1
            self.get_logger().debug(
                f'LEFT Obstacle detected! distance to obstacle: {right_dist:.2f} meters. Turning at Angular velocity: {self.twist.angular.z} rad/s. current score:{self.scores}')
        else:
            self.twist.linear.x = 0.5  # Move forward
            self.twist.angular.z = 0.0  # No rotation
            self.scores += 3
            self.get_logger().debug(
                f'Path clear. Moving forward. Front distance: {front_dist:.2f} meters. current score:{self.scores}')

        self.pub.publish(self.twist)

    def reset_simulation(self):
        self.get_logger().debug("Resetting simulation...")
        self.reset_triggered = True

    def crash_reset_simulation(self):
        self.get_logger().debug("Resetting simulation...")
        client = self.create_client(Empty, '/reset_simulation')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Waiting for reset_simulation service...')

        request = Empty.Request()
        future = client.call_async(request)
        self.reset_triggered = True


def initialize_population(population_size):
    std_dev = 0.1
    fast_turn = left_turn = right_turn = 0
    population_list = []
    individual = dict()
    for _ in range(population_size):
        fast_turn = 0.8 + random.gauss(0, std_dev)
        left_turn = 0.4 + random.gauss(0, std_dev)
        right_turn = -0.4 + random.gauss(0, std_dev)
        individual['fast_turn'] = fast_turn
        individual['left_turn'] = left_turn
        individual['right_turn'] = right_turn
        population_list.append(individual)
    return population_list


def evolve_population(population, scores, population_size, mutation_rate):
    top_performers = np.argsort(scores)[-population_size // 2:]
    # select from the best of the population to form parent population
    offspring_population = [population[i] for i in top_performers]
    while len(offspring_population) < population_size:
        # randomly choose from the parent population to form offspring
        parent1 = random.choice(offspring_population)
        parent2 = random.choice(offspring_population)
        # using cross-over
        child = crossover(parent1, parent2)
        offspring_population.append(child)
    # mutation
    population = [mutate(ind, mutation_rate) for ind in offspring_population]
    return population


def crossover(parent1, parent2):
    fast_turn = (parent1['fast_turn'] + parent2['fast_turn']) / 2
    left_turn = (parent1['left_turn'] + parent2['left_turn']) / 2
    right_turn = (parent1['right_turn'] + parent2['right_turn']) / 2

    return {
        'fast_turn': fast_turn,
        'left_turn': left_turn,
        'right_turn': right_turn
    }


# to mutate: use gaussian noise
def mutate(individual, mutation_rate):
    individual['fast_turn'] += random.gauss(0, mutation_rate)
    individual['left_turn'] += random.gauss(0, mutation_rate)
    individual['right_turn'] += random.gauss(0, mutation_rate)
    return individual


def main(args=None):
    rclpy.init()
    population_size = 6
    generations = 10
    mutation_rate = 0.05
    population = []
    best_scores = []
    for current_generation in range(generations):
        # Initialize or evolve population
        print(f'Generation {current_generation + 1}:')
        population = initialize_population(population_size) if current_generation == 0 \
            else evolve_population(population, scores, population_size, mutation_rate)

        # Track scores for the current generation
        scores = []

        for i in range(population_size):
            current_individual = population[i]
            node = EvolutionaryObstacleAvoidance(current_individual)
            try:
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if node.reset_triggered:  # Break the spin loop if reset is triggered
                        scores.append(node.scores)  # Append the scores after the reset
                        print(f'population {i + 1} has score: {node.scores}')
                        node.get_logger().debug('Reset detected, proceeding.')
                        break
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()

        # Find the maximum score and corresponding individual
        max_score = max(scores)
        best_scores.append(max_score)
        best_individual = population[scores.index(max_score)]

        # Print the best score and motor speed pairs for every five generations
        if current_generation == 0 or (current_generation + 1) % 5 == 0:
            print(f'Maximum Score: {max_score}')
            print(f'Best Individual Motor Speeds:')
            print(f'  Fast Turn Speed: {best_individual["fast_turn"]:.2f} rad/s')
            print(f'  Left Turn Speed: {best_individual["left_turn"]:.2f} rad/s')
            print(f'  Right Turn Speed: {best_individual["right_turn"]:.2f} rad/s')
    print(f' Score history for all generations are {best_scores}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
