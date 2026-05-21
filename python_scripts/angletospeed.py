import rclpy
import time
from rclpy.node import Node
from mirte_msgs.srv import SetServoAngle 

class MirteController(Node):
    def __init__(self):
        super().__init__('mirte_timed_controller')
        self.steer_client = self.create_client(SetServoAngle, '/io/servo/stuur/set_angle')
        self.gas_client = self.create_client(SetServoAngle, '/io/servo/gas/set_angle')

        # Wait for services
        while not self.steer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for services...')

    def send_command(self, steer, gas):
        # Create requests
        s_req = SetServoAngle.Request()
        s_req.angle = float(steer)
        g_req = SetServoAngle.Request()
        g_req.angle = float(gas)

        # Call services (asynchronous)
        self.steer_client.call_async(s_req)
        self.gas_client.call_async(g_req)
        self.get_logger().info(f'Sent: Steer {steer}, Gas {gas}')

def main(args=None):
    rclpy.init(args=args)
    node = MirteController()

    # --- ACTION SEQUENCE ---
    # 1. Start Driving (e.g., Steer straight at 90, Gas at 110)
   

    node.send_command(90, 90)
    time.sleep(3.0)
    node.send_command(117, 90)
    time.sleep(3.0)
    node.send_command(125, 90)
    time.sleep(3.0)
    

    

    # 2. Wait for a specific time (e.g., 3 seconds)
    # We use node.get_clock().spin_until_future_complete or a simple sleep for scripts

    
    # 3. Stop Driving (Set Gas back to neutral, e.g., 90)
    node.get_logger().info('Time up! Stopping...')
    #node.send_command(94, 90)

    # Give ROS a moment to actually send the stop command before shutting down
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()