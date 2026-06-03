#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


def euler_from_quaternion(q):
    """Zet quaternion [x, y, z, w] om naar euler hoeken (yaw, pitch, roll)."""
    x, y, z, w = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    return yaw, pitch, roll


class BicycleBEPTest(Node):

    def __init__(self):
        super().__init__('bicycle_bep_test')

        #
        # Publisher (Aangepast naar jouw topic!)
        #
        self.pub_reference = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            10
        )

        #
        # Subscriber (Aangepast naar jouw topic!)
        #
        self.create_subscription(
            Odometry,
            '/bicycle_steering_controller/odometry',
            self.odom_callback,
            10
        )

        # PD controller settings
        self.Kp = 2.0
        self.Kd = 0.5
        self.max_yaw_rate = 0.5

        self.last_time = None

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.w = 0.0

        self.get_logger().info('Bicycle BEP test node gestart!')

    def run_test(self):
        self.get_logger().info("Wachten op eerste odometry...")
        time.sleep(2.0)

        # In deze opzet sturen we echte snelheden (m/s) en hoeken/hoeksnelheden (rad/s)
        
        # 1. Stukje rechtuit (Snelheid: 0.2 m/s, Sturen: 0.0 rad/s)
        self.get_logger().info("Rij rechtuit...")
        self.drive(linear_x=1.868, angular_z=0.0, dur=1.5)

        # 2. Flauwe bocht maken (Snelheid: 0.2 m/s, Sturen: 0.3 rad/s)
        self.get_logger().info("Maak een bocht...")
        self.drive(linear_x=1.868, angular_z=10.0, dur=2.0)

        # 3. Stoppen
        self.get_logger().info("Stop de robot.")
        self.drive(linear_x=0.0, angular_z=0.0, dur=0.5)

        print("\nEindpositie volgens Odometry:")
        print(f"x: {self.x:.3f} m")
        print(f"y: {self.y:.3f} m")
        print(f"theta (yaw): {self.theta:.3f} rad")

    def drive(self, linear_x, angular_z, dur):
        rate_hz = 30.0  # Gelijk aan de 30Hz uit jouw handmatige pub command
        period = 1.0 / rate_hz

        start_time = time.time()
        end_time = start_time + dur
        self.last_time = time.time()

        while time.time() < end_time and rclpy.ok():
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            if dt <= 0.0:
                dt = 0.033

            # PD correctie als we rechtuit willen (angular_z is bijna 0)
            if abs(linear_x) > 0.0 and abs(angular_z) < 0.01:
                # Houd de huidige hoek vast (simpele koerscorrectie)
                yaw_error = 0.0 - self.theta  # Ervan uitgaande dat 0.0 rechtuit is
                yaw_error = (yaw_error + math.pi) % (2.0 * math.pi) - math.pi

                p_term = self.Kp * yaw_error
                d_term = -self.Kd * self.w
                target_angular_z = p_term + d_term
            else:
                target_angular_z = angular_z

            # Bouw het TwistStamped bericht op
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'  # Of 'odom', afhankelijk van je controller eisen
            
            # Snelheid (m/s)
            msg.twist.linear.x = float(linear_x)
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            
            # Sturen / Draaisnelheid (rad/s)
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = float(target_angular_z)

            self.pub_reference.publish(msg)
            time.sleep(period)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        yaw, _, _ = euler_from_quaternion(q)
        
        self.theta = yaw
        self.w = msg.twist.twist.angular.z


def main(args=None):
    rclpy.init(args=args)
    node = BicycleBEPTest()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_test()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()