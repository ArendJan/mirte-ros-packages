#!/usr/bin/env python3
import sys
import sys
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
print("Hello from Python script!")
# print cwd
print("Current working directory:", os.getcwd())
# SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(os.path.dirname(SCRIPT_DIR))

sys.path.append('/home/arendjan/ros2/mirte_ros2_ws/src/mirte-ros-packages/mirte_telemetrix_cpp/build')
print("je moeder")
result = 21
import        hello_world
# print(hello_world.hello_world)
x = hello_world.World()
x.set("test")
print(x.greet())


# multiprocessing test
import multiprocessing
def worker():
    print("Worker process started")
    for i in range(5):
        print(f"Worker iteration {i}")
        # rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(1)
    print("Worker process finished")

# if __name__ == '__main__':
print("Starting worker process")
p = multiprocessing.Process(target=worker)
p.start()
# p.join()
print("Worker process joined")

# test node
rclpy.init()
node = Node("test_node")
node.get_logger().info("Hello from ROS 2 node!")
# create a publisher
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
publisher = node.create_publisher(String, 'test_topic', qos_profile)
# create timer
timer_period = 1.0  # seconds
timer = node.create_timer(timer_period, lambda: publisher.publish(String(data="Hello from timer!")))
# spin node
# try:
#     rclpy.spin(node)
# except KeyboardInterrupt:
#     node.get_logger().info("Node stopped by user.")
# finally:
#     node.destroy_timer(timer)
#     node.destroy_node()
# rclpy.shutdown()


def loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        # return timer.time_since_last_call()
        # node.get_logger().info("Timer callback executed!")
        # rclpy.sleep(1.0)

def upd(x):
    print("Updating x:", x)
    return x + 1