#!/usr/bin/env python3

import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32, Float32MultiArray


class LatencyMeasurer(Node):

    def __init__(self, car_number):
        super().__init__('latency_measurer_node')

        self.car_number = str(car_number)

        # Testinstellingen
        self.test_speed = 0.2
        self.test_iterations = 5

        # Timing
        self.cmd_sent_time = None
        self.waiting_for_echo = False

        self.latency_measurements = []

        # Subscribers
        self.throttle_sub = self.create_subscription(
            Float32,
            f'throttle_{self.car_number}',
            self.throttle_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            Float32MultiArray,
            f'arduino_data_{self.car_number}',
            self.arduino_callback,
            10
        )

        # Publishers
        self.throttle_pub = self.create_publisher(
            Float32,
            f'throttle_{self.car_number}',
            10
        )

        self.safety_pub = self.create_publisher(
            Float32,
            'safety_value',
            10
        )

        self.get_logger().info(
            f'Latency Measurer gestart voor auto {self.car_number}'
        )

    def throttle_callback(self, msg):
        """
        Wordt aangeroepen zodra een throttlebericht
        op de topic verschijnt.
        """

        if not self.waiting_for_echo:
            self.cmd_sent_time = self.get_clock().now()
            self.waiting_for_echo = True

            self.get_logger().debug(
                'Throttle commando gedetecteerd. Timer gestart.'
            )

    def arduino_callback(self, msg):
        """
        Wordt aangeroepen zodra encoderdata binnenkomt.
        """

        if self.waiting_for_echo and self.cmd_sent_time is not None:

            arrival_time = self.get_clock().now()

            latency_ns = (
                arrival_time - self.cmd_sent_time
            ).nanoseconds

            latency_ms = latency_ns / 1e6

            self.latency_measurements.append(latency_ms)

            self.get_logger().info(
                f'Latency gedetecteerd: {latency_ms:.2f} ms'
            )

            self.waiting_for_echo = False

    def run_test(self):

        # Even wachten zodat connecties kunnen opzetten
        time.sleep(1.0)

        self.get_logger().info('Start latency test')

        for i in range(self.test_iterations):

            self.get_logger().info(
                f'Meting {i + 1}/{self.test_iterations}'
            )

            # Safety vrijgeven
            safety_msg = Float32()
            safety_msg.data = 1.0
            self.safety_pub.publish(safety_msg)

            # Throttle aan
            throttle_msg = Float32()
            throttle_msg.data = self.test_speed
            self.throttle_pub.publish(throttle_msg)

            time.sleep(0.5)

            # Throttle uit
            throttle_msg.data = 0.0
            self.throttle_pub.publish(throttle_msg)

            time.sleep(0.5)

        self.get_logger().info('Test afgerond')

        print("\nLatency resultaten:")
        print(self.latency_measurements)

        if len(self.latency_measurements) > 0:
            print(
                "Gemiddelde latency over {} metingen: {:.2f} ms".format(
                    len(self.latency_measurements),
                    np.mean(self.latency_measurements)
                )
            )
        else:
            print("Geen latency-metingen ontvangen.")


def main(args=None):

    rclpy.init(args=args)

    try:
        car_number = os.environ.get('car_number', '1')

        node = LatencyMeasurer(car_number)

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        import threading

        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True
        )
        spin_thread.start()

        node.run_test()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()