#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
from datetime import datetime

class WorkspaceAnalyzer(Node):
    def __init__(self):
        super().__init__('workspace_analyzer')
        
        # DH parameters and link lengths (from URDF)
        self.link_lengths = {
            'base_height': 0.6,
            'link1': 0.15,
            'link2': 0.4,
            'link3': 0.35,
            'link4': 0.075,
            'link5': 0.12,
            'instrument': 0.54
        }
        
        # Arm base positions (Compact config)
        self.arm_positions = {
            'arm1': {'x': 0.5, 'y': -1.4, 'angle': math.radians(100)},
            'arm2': {'x': 0.0, 'y': -1.5, 'angle': math.radians(140)},
            'arm3': {'x': -0.5, 'y': 1.4, 'angle': math.radians(220)},
            'arm4': {'x': 0.5, 'y': 1.4, 'angle': math.radians(260)},
        }
        
        # Port positions (Straight config)
        self.ports_straight = {
            'port1': np.array([0.08, -0.05, 0.9]),   # Right lateral
            'port2': np.array([0.0, 0.0, 0.9]),      # Camera (umbilical)
            'port3': np.array([-0.08, -0.05, 0.9]),  # Left lateral
            'port4': np.array([0.08, 0.05, 0.9]),    # Right flank
        }
        
        # Port positions (Bridge config - 4cm lower)
        self.ports_bridge = {
            'port1': np.array([0.08, -0.09, 0.9]),
            'port2': np.array([0.0, 0.0, 0.9]),  # Camera unchanged
            'port3': np.array([-0.08, -0.09, 0.9]),
            'port4': np.array([0.08, 0.05, 0.9]),
        }
        
        self.get_logger().info('Starting workspace analysis...')
        self.analyze_workspace()
        
    def forward_kinematics(self, arm_name, joint_angles):
        """
        Compute end-effector position given joint angles
        Simplified FK using link lengths
        """
        base = self.arm_positions[arm_name]
        
        # Start at arm base
        x = base['x']
        y = base['y']
        z = self.link_lengths['base_height']
        
        # Joint 1: Base rotation
        angle_total = joint_angles[0]
        
        # Joint 2: Shoulder pitch
        z += self.link_lengths['link1']
        x += self.link_lengths['link2'] * math.cos(angle_total) * math.cos(joint_angles[1])
        y += self.link_lengths['link2'] * math.sin(angle_total) * math.cos(joint_angles[1])
        z += self.link_lengths['link2'] * math.sin(joint_angles[1])
        
        # Joint 3: Elbow
        x += self.link_lengths['link3'] * math.cos(angle_total) * math.cos(joint_angles[1] + joint_angles[2])
        y += self.link_lengths['link3'] * math.sin(angle_total) * math.cos(joint_angles[1] + joint_angles[2])
        z += self.link_lengths['link3'] * math.sin(joint_angles[1] + joint_angles[2])
        
        # Wrist joints (simplified - just add remaining length)
        total_wrist_length = self.link_lengths['link4'] + self.link_lengths['link5'] + self.link_lengths['instrument']
        x += total_wrist_length * math.cos(angle_total) * math.cos(joint_angles[1] + joint_angles[2])
        y += total_wrist_length * math.sin(angle_total) * math.cos(joint_angles[1] + joint_angles[2])
        z += total_wrist_length * math.sin(joint_angles[1] + joint_angles[2])
        
        return np.array([x, y, z])
    
    def sample_workspace(self, arm_name, num_samples=1000):
        """
        Sample workspace by trying random joint configurations
        """
        points = []
        
        # Joint limits (from URDF)
        limits = [
            (-3.14, 3.14),   # Joint 1
            (-1.57, 1.57),   # Joint 2
            (-2.0, 2.0),     # Joint 3
            (-3.14, 3.14),   # Joint 4
            (-1.57, 1.57),   # Joint 5
            (-3.14, 3.14),   # Joint 6
        ]
        
        for _ in range(num_samples):
            # Random joint angles within limits
            joints = [np.random.uniform(low, high) for low, high in limits]
            
            # Compute FK
            point = self.forward_kinematics(arm_name, joints)
            points.append(point)
        
        return np.array(points)
    
    def check_port_reachability(self, workspace_points, port_position, tolerance=0.05):
        """
        Check if port is within reachable workspace
        """
        distances = np.linalg.norm(workspace_points - port_position, axis=1)
        min_distance = np.min(distances)
        is_reachable = min_distance < tolerance
        
        return is_reachable, min_distance
    
    def check_arm_collisions(self, workspaces):
        """
        Check if arm workspaces overlap (potential collision)
        """
        collisions = []
        arms = list(workspaces.keys())
        
        for i in range(len(arms)):
            for j in range(i+1, len(arms)):
                arm1, arm2 = arms[i], arms[j]
                ws1, ws2 = workspaces[arm1], workspaces[arm2]
                
                # Simple check: minimum distance between workspace points
                min_dist = float('inf')
                for p1 in ws1[::10]:  # Sample every 10th point for speed
                    for p2 in ws2[::10]:
                        dist = np.linalg.norm(p1 - p2)
                        min_dist = min(min_dist, dist)
                
                if min_dist < 0.1:  # 10cm threshold
                    collisions.append({
                        'arms': f'{arm1}-{arm2}',
                        'min_distance': min_dist
                    })
        
        return collisions
    
    def analyze_workspace(self):
        """
        Main analysis function
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = f'/home/sumanthmurthy/workspace_analysis_{timestamp}.csv'
        
        self.get_logger().info('Computing workspace for each arm...')
        
        # Sample workspace for all arms
        workspaces = {}
        for arm in ['arm1', 'arm2', 'arm3', 'arm4']:
            self.get_logger().info(f'Sampling {arm}...')
            workspaces[arm] = self.sample_workspace(arm, num_samples=2000)
        
        # Analyze port reachability
        self.get_logger().info('\n=== STRAIGHT PORT PLACEMENT ===')
        straight_results = []
        
        port_arm_mapping = {
            'port1': 'arm1',
            'port2': 'arm2',
            'port3': 'arm3',
            'port4': 'arm4'
        }
        
        for port_name, arm_name in port_arm_mapping.items():
            port_pos = self.ports_straight[port_name]
            reachable, distance = self.check_port_reachability(
                workspaces[arm_name], port_pos
            )
            
            result = {
                'config': 'Straight',
                'port': port_name,
                'arm': arm_name,
                'reachable': reachable,
                'min_distance': distance,
                'port_x': port_pos[0],
                'port_y': port_pos[1],
                'port_z': port_pos[2]
            }
            straight_results.append(result)
            
            self.get_logger().info(
                f'{port_name} ({arm_name}): {"REACHABLE" if reachable else "NOT REACHABLE"} '
                f'(min dist: {distance:.3f}m)'
            )
        
        # Analyze bridge config
        self.get_logger().info('\n=== BRIDGE PORT PLACEMENT ===')
        bridge_results = []
        
        for port_name, arm_name in port_arm_mapping.items():
            port_pos = self.ports_bridge[port_name]
            reachable, distance = self.check_port_reachability(
                workspaces[arm_name], port_pos
            )
            
            result = {
                'config': 'Bridge',
                'port': port_name,
                'arm': arm_name,
                'reachable': reachable,
                'min_distance': distance,
                'port_x': port_pos[0],
                'port_y': port_pos[1],
                'port_z': port_pos[2]
            }
            bridge_results.append(result)
            
            self.get_logger().info(
                f'{port_name} ({arm_name}): {"REACHABLE" if reachable else "NOT REACHABLE"} '
                f'(min dist: {distance:.3f}m)'
            )
        
        # Check collisions
        self.get_logger().info('\n=== COLLISION ANALYSIS ===')
        collisions = self.check_arm_collisions(workspaces)
        
        if collisions:
            for col in collisions:
                self.get_logger().warn(
                    f'Potential collision: {col["arms"]} (min distance: {col["min_distance"]:.3f}m)'
                )
        else:
            self.get_logger().info('No workspace collisions detected')
        
        # Workspace volumes
        self.get_logger().info('\n=== WORKSPACE VOLUMES ===')
        for arm, points in workspaces.items():
            # Approximate volume using convex hull points
            x_range = np.max(points[:, 0]) - np.min(points[:, 0])
            y_range = np.max(points[:, 1]) - np.min(points[:, 1])
            z_range = np.max(points[:, 2]) - np.min(points[:, 2])
            volume = x_range * y_range * z_range
            
            self.get_logger().info(f'{arm}: ~{volume:.3f} m³')
        
        # Save results
        with open(results_file, 'w', newline='') as f:
            fieldnames = ['config', 'port', 'arm', 'reachable', 'min_distance', 
                         'port_x', 'port_y', 'port_z']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(straight_results + bridge_results)
        
        self.get_logger().info(f'\nResults saved to: {results_file}')
        self.get_logger().info('Workspace analysis complete!')
        
        rclpy.shutdown()

def main():
    rclpy.init()
    node = WorkspaceAnalyzer()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()