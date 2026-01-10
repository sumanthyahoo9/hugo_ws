#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import math
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        # Publisher for joint commands
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        
        # Subscriber to actual joint states
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_commands)
        
        # CSV file setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f'/home/sumanthmurthy/trajectory_{timestamp}.csv'
        self.csv_writer = None
        self.csv_handle = open(self.csv_file, 'w', newline='')
        
        # Joint names
        self.joint_names = []
        for arm in ['arm1', 'arm2', 'arm3', 'arm4']:
            for joint_num in range(1, 7):
                self.joint_names.append(f'{arm}_joint{joint_num}')
        
        # Initialize CSV
        header = ['timestamp'] + self.joint_names
        self.csv_writer = csv.writer(self.csv_handle)
        self.csv_writer.writerow(header)
        
        # Simulation parameters
        self.start_time = self.get_clock().now()
        self.duration = 600  # seconds
        
        self.get_logger().info(f'Recording to: {self.csv_file}')
        self.get_logger().info(f'Duration: {self.duration}s')
    
    def state_callback(self, msg):
        """Log actual joint states to CSV"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds * 1e-9
        
        # Stop after duration
        if elapsed > self.duration:
            self.get_logger().info('Recording complete!')
            self.csv_handle.close()
            rclpy.shutdown()
            return
        
        # Write to CSV
        row = [elapsed] + list(msg.position)
        self.csv_writer.writerow(row)
    
    def publish_commands(self):
        """Publish joint commands - simulates docking motion"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds * 1e-9
        
        # Stop commanding after duration
        if t > self.duration:
            return
        
        # Trajectory: Simulate arms moving to docking positions
        # Arm 1: Rotate base and move to position
        arm1_base_rotation = 0.5 * math.sin(t * 0.3)
        arm1_shoulder = -0.3 * math.sin(t * 0.5)
        
        # Arm 2 (camera): Different motion
        arm2_base_rotation = 0.3 * math.sin(t * 0.4)
        
        # Arm 3 & 4: Slow synchronized motion
        arm34_motion = 0.4 * math.sin(t * 0.2)
        
        msg.position = [
            # Arm 1
            arm1_base_rotation, arm1_shoulder, 0.2, 0.0, 0.0, 0.0,
            # Arm 2
            arm2_base_rotation, -0.2, 0.0, 0.0, 0.0, 0.0,
            # Arm 3
            arm34_motion, 0.0, 0.0, 0.0, 0.0, 0.0,
            # Arm 4
            arm34_motion, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
        
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_handle.close()
        node.destroy_node()

if __name__ == '__main__':
    main()