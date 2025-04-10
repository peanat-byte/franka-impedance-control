#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from franka_msgs.msg import FrankaState
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import csv
import os
from datetime import datetime

class TestLogging:
    def __init__(self, max_data_points=10000):
        # Initialize ROS node
        rospy.init_node('test_logging', anonymous=True)
        
        # Data storage
        self.max_data_points = max_data_points
    
        self.time_data = deque(maxlen=max_data_points)
        self.time_data2 = deque(maxlen=max_data_points)
        self.time_data3 = deque(maxlen=max_data_points)
        
        # Force data
        self.fx_data = deque(maxlen=max_data_points)
        self.fy_data = deque(maxlen=max_data_points)
        self.fz_data = deque(maxlen=max_data_points)
        
        # Position data (actual)
        self.x_data = deque(maxlen=max_data_points)
        self.y_data = deque(maxlen=max_data_points)
        self.z_data = deque(maxlen=max_data_points)
        
        # Position data (desired)
        self.x_desired = deque(maxlen=max_data_points)
        self.y_desired = deque(maxlen=max_data_points)
        self.z_desired = deque(maxlen=max_data_points)
        
        # Start time
        self.start_time = time.time()
        
        # Subscribers
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.force_callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_callback)
        rospy.Subscriber("/impedance_controller_mech_464/equilibrium_pose", PoseStamped, self.equilibrium_pose_callback)
        
        # Register shutdown handler
        rospy.on_shutdown(self.save_data)
        
    def force_callback(self, msg):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.fx_data.append(msg.wrench.force.x)
        self.fy_data.append(msg.wrench.force.y)
        self.fz_data.append(msg.wrench.force.z)
        
    def state_callback(self, msg):
        current_time = time.time() - self.start_time
        self.time_data2.append(current_time)
        # O_T_EE is 4x4 transformation matrix
        O_T_EE = np.array(msg.O_T_EE)
        self.x_data.append(O_T_EE[12])
        self.y_data.append(O_T_EE[13])
        self.z_data.append(O_T_EE[14])

    def equilibrium_pose_callback(self, msg):
        current_time = time.time() - self.start_time
        self.time_data3.append(current_time)
        self.x_desired.append(msg.pose.position.x)
        self.y_desired.append(msg.pose.position.y)
        self.z_desired.append(msg.pose.position.z)
    
    def save_data(self):
        """Save all data to CSV and generate plots when exiting"""
        print("\nSaving data and generating plots...")
        
        # Create a directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dir_name = f"franka_data_{timestamp}"
        os.makedirs(dir_name, exist_ok=True)
        
        # Save data to CSV
        csv_filename = os.path.join(dir_name, "data.csv")
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow([
                'time', 
                'fx', 'fy', 'fz',
                'time2',
                'x_actual', 'y_actual', 'z_actual',
                'time3',
                'x_desired', 'y_desired', 'z_desired'
            ])
            
            # Write data
            max_length = max(
                len(self.time_data),
                len(self.fx_data),
                len(self.time_data2),
                len(self.x_data),
                len(self.time_data3),
                len(self.x_desired),
            )
            
            for i in range(max_length):
                row = [
                    self.time_data[i] if i < len(self.time_data) else '',
                    self.fx_data[i] if i < len(self.fx_data) else '',
                    self.fy_data[i] if i < len(self.fy_data) else '',
                    self.fz_data[i] if i < len(self.fz_data) else '',
                    self.time_data2[i] if i < len(self.time_data2) else '',
                    self.x_data[i] if i < len(self.x_data) else '',
                    self.y_data[i] if i < len(self.y_data) else '',
                    self.z_data[i] if i < len(self.z_data) else '',
                    self.time_data3[i] if i < len(self.time_data2) else '',
                    self.x_desired[i] if i < len(self.x_desired) else '',
                    self.y_desired[i] if i < len(self.y_desired) else '',
                    self.z_desired[i] if i < len(self.z_desired) else '',
                ]
                writer.writerow(row)
        
        print(f"Data saved to {csv_filename}")
        
        # Generate and save plots
        self.generate_plots(dir_name)
    
    def generate_plots(self, output_dir):
        """Generate and save plots"""
        plt.figure(figsize=(12, 8))
        # plt.suptitle('Franka Panda State Monitoring', fontsize=14)
        
        # Force plot
        plt.subplot(3, 1, 1)
        plt.plot(self.time_data, self.fx_data, 'r-', label='Fx')
        plt.plot(self.time_data, self.fy_data, 'g-', label='Fy')
        plt.plot(self.time_data, self.fz_data, 'b-', label='Fz')
        plt.title('External Forces')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.grid(True)
        
        # Position plot
        plt.subplot(3, 1, 2)
        plt.plot(self.time_data2, self.x_data, 'r-', label='X actual')
        plt.plot(self.time_data2, self.y_data, 'g-', label='Y actual')
        plt.plot(self.time_data2, self.z_data, 'b-', label='Z actual')
    
        plt.plot(self.time_data3, self.x_desired, 'r--', label='X desired')
        plt.plot(self.time_data3, self.y_desired, 'g--', label='Y desired')
        plt.plot(self.time_data3, self.z_desired, 'b--', label='Z desired')
        
        plt.title('End-Effector Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save the figure
        plot_filename = os.path.join(output_dir, "franka_state_plots.png")
        plt.savefig(plot_filename)
        print(f"Plots saved to {plot_filename}")
        
        # Optionally show the plot (comment out if running headless)
        # plt.show()
        plt.close()
    
    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    logger = TestLogging()
    logger.run()