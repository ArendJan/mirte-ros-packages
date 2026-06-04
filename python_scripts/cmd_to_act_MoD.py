#!/usr/bin/env python3

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped

from mirte_msgs.msg import Encoder


class LatencyMeasurer(Node):

    def __init__(self):
        super().__init__('latency_measurer')

        # Test settings
        self.test_speed = 1.3
        self.test_iterations = 10

        # Measurement state
        self.cmd_sent_time = None
        self.waiting_for_motion = False

        self.latency_measurements = []

        # Encoder tracking
        self.last_encoder_value = None

        #
        # Publisher
        #
        self.reference_pub = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            10
        )

        #
        # Controller reference subscriber
        #
        self.create_subscription(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            self.reference_callback,
            10
        )

        #
        # Encoder subscriber
        #
        self.create_subscription(
            Encoder,
            '/io/encoder/main',
            self.encoder_callback,
            10
        )

        self.get_logger().info('Latency measurer started')

    def reference_callback(self, msg):

        throttle = msg.twist.linear.x

        if not self.waiting_for_motion and throttle > 0.0:

            self.cmd_sent_time = self.get_clock().now()
            self.waiting_for_motion = True

            self.get_logger().info(
                f'Throttle detected: {throttle:.3f}'
            )

    def encoder_callback(self, msg):

        current_value = msg.value

        # Debug
        self.get_logger().debug(
            f'Encoder value: {current_value}'
        )

        if self.last_encoder_value is None:
            self.last_encoder_value = current_value
            return

        if (
            self.waiting_for_motion
            and current_value != self.last_encoder_value
        ):

            arrival_time = self.get_clock().now()

            latency_ms = (
                arrival_time - self.cmd_sent_time
            ).nanoseconds / 1e6

            self.latency_measurements.append(latency_ms)

            self.get_logger().info(
                f'Encoder changed: {self.last_encoder_value} -> {current_value}'
            )

            self.get_logger().info(
                f'Latency: {latency_ms:.2f} ms'
            )

            self.waiting_for_motion = False

        self.last_encoder_value = current_value

    def publish_reference(self, throttle):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = float(throttle)
        msg.twist.angular.z = 0.0

        self.reference_pub.publish(msg)

    def stop_robot(self):
        self.publish_reference(0.0)

    def run_test(self):

        self.get_logger().info(
            'Waiting for connections...'
        )

        time.sleep(2.0)

        for i in range(self.test_iterations):

            self.get_logger().info(
                f'Test {i + 1}/{self.test_iterations}'
            )

            self.publish_reference(self.test_speed)

            time.sleep(0.5)

            self.stop_robot()

            time.sleep(4.0)

        print('\n==============================')
        print('LATENCY RESULTS')
        print('==============================')

        for i, latency in enumerate(self.latency_measurements):
            print(
                f'Measurement {i+1}: {latency:.2f} ms'
            )

        if len(self.latency_measurements) > 0:

            avg_latency = np.mean(
                self.latency_measurements
            )

            print(
                f'\nAverage latency ({len(self.latency_measurements)} measurements): '
                f'{avg_latency:.2f} ms'
            )

        else:
            print('No latency measurements recorded.')

        self.stop_robot()


def main(args=None):

    rclpy.init(args=args)

    node = LatencyMeasurer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )
    spin_thread.start()

    try:
        node.run_test()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()