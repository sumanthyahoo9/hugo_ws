#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        # Subscribe to commands
        self.cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.cmd_callback, 10)
        
        # Publish to joint states (simulates actuators)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.get_logger().info('Simple controller started - bridging commands to states')
    
    def cmd_callback(self, msg):
        """Simply forward commands as states (perfect tracking)"""
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()