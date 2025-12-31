#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class DockingSequence(Node):
    def __init__(self):
        super().__init__('docking_sequence')
        
        # Declare parameter for configuration type
        self.declare_parameter('config', 'compact')  # 'compact' or 'butterfly'
        
        self.config = self.get_parameter('config').value
        self.get_logger().info(f'Starting docking sequence: {self.config.upper()} configuration')
        
        # Publisher
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_docking)
        
        # Joint names
        self.joint_names = []
        for arm in ['arm1', 'arm2', 'arm3', 'arm4']:
            for joint_num in range(1, 7):
                self.joint_names.append(f'{arm}_joint{joint_num}')
        
        # Docking configurations from paper (in radians)
        # Compact: Arm angles [100°, 140°, 220°, 260°]
        # Butterfly: Arm angles [90°, 140°, 220°, 270°]
        
        self.compact_config = {
            'arm1': {'base_angle': math.radians(100), 'tilt': math.radians(15)},
            'arm2': {'base_angle': math.radians(140), 'tilt': math.radians(-30)},
            'arm3': {'base_angle': math.radians(220), 'tilt': math.radians(-30)},
            'arm4': {'base_angle': math.radians(260), 'tilt': math.radians(15)},
        }
        
        self.butterfly_config = {
            'arm1': {'base_angle': math.radians(90), 'tilt': math.radians(15)},
            'arm2': {'base_angle': math.radians(140), 'tilt': math.radians(-30)},
            'arm3': {'base_angle': math.radians(220), 'tilt': math.radians(-30)},
            'arm4': {'base_angle': math.radians(270), 'tilt': math.radians(15)},
        }
        
        # Select configuration
        self.target_config = self.compact_config if self.config == 'compact' else self.butterfly_config
        
        # Motion parameters
        self.start_time = self.get_clock().now()
        self.docking_duration = 10.0  # seconds to reach position
        self.hold_duration = 5.0      # seconds to hold position
        self.total_duration = self.docking_duration + self.hold_duration
        
    def publish_docking(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds * 1e-9
        
        # Stop after total duration
        if elapsed > self.total_duration:
            self.get_logger().info('Docking sequence complete!')
            rclpy.shutdown()
            return
        
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.joint_names
        
        # Smooth transition using sigmoid function
        if elapsed < self.docking_duration:
            progress = elapsed / self.docking_duration
            # Smooth S-curve: 0 to 1
            smooth_progress = 3 * progress**2 - 2 * progress**3
        else:
            smooth_progress = 1.0
        
        # Build position array
        positions = []
        
        for arm_name in ['arm1', 'arm2', 'arm3', 'arm4']:
            config = self.target_config[arm_name]
            
            # Joint 1: Base rotation (approach angle)
            positions.append(config['base_angle'] * smooth_progress)
            
            # Joint 2: Shoulder pitch (tilt)
            positions.append(config['tilt'] * smooth_progress)
            
            # Joint 3: Elbow (extend arm toward patient)
            positions.append(0.5 * smooth_progress)
            
            # Joints 4-6: Wrist positioning for instrument alignment
            positions.append(0.0)  # Wrist roll
            positions.append(-0.3 * smooth_progress)  # Wrist pitch (point downward)
            positions.append(0.0)  # Wrist yaw
        
        msg.position = positions
        self.cmd_pub.publish(msg)
        
        # Log progress
        if int(elapsed) != int(elapsed - 0.1):  # Log every second
            self.get_logger().info(f'Progress: {smooth_progress*100:.0f}%')

def main():
    rclpy.init()
    node = DockingSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()