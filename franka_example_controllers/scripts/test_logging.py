#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
from matplotlib.widgets import Button

# import os
# import matplotlib

# # Check if we're in WSL
# if 'microsoft' in os.uname().release.lower():
#     # Use 'Agg' backend if no display is available
#     try:
#         import tkinter
#         matplotlib.use('TkAgg')  # Try Tk first
#     except:
#         matplotlib.use('Agg')  # Non-interactive backend
#     # Set display if not set
#     if not os.environ.get('DISPLAY'):
#         os.environ['DISPLAY'] = ':0'


class TestLogging:
    def __init__(self, max_data_points=200):
        # Initialize ROS node
        rospy.init_node('test_logging', anonymous=True)
        
        # Data storage
        self.max_data_points = max_data_points
        self.time_data = deque(maxlen=max_data_points)
        
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
        
        # Position error
        self.x_error = deque(maxlen=max_data_points)
        self.y_error = deque(maxlen=max_data_points)
        self.z_error = deque(maxlen=max_data_points)
        
        # Initialize plots
        plt.ion()
        self.fig = plt.figure(figsize=(12, 8))
        self.fig.suptitle('Franka Panda State Monitoring', fontsize=14)
        
        # Create subplots
        self.ax_force = plt.subplot2grid((3, 2), (0, 0))
        self.ax_position = plt.subplot2grid((3, 2), (1, 0))
        self.ax_error = plt.subplot2grid((3, 2), (2, 0))
        
        # Initialize lines
        self.init_force_plot()
        self.init_position_plot()
        self.init_error_plot()
        
        # Add clear button
        ax_clear = plt.axes([0.81, 0.01, 0.1, 0.05])
        self.btn_clear = Button(ax_clear, 'Clear Data')
        self.btn_clear.on_clicked(self.clear_data)
        
        # Start time
        self.start_time = time.time()
        
        # Subscribers
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.force_callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_callback)
        
    def init_force_plot(self):
        self.line_fx, = self.ax_force.plot([], [], 'r-', label='Fx')
        self.line_fy, = self.ax_force.plot([], [], 'g-', label='Fy')
        self.line_fz, = self.ax_force.plot([], [], 'b-', label='Fz')
        self.ax_force.set_title('External Forces')
        self.ax_force.set_xlabel('Time (s)')
        self.ax_force.set_ylabel('Force (N)')
        self.ax_force.legend()
        self.ax_force.grid(True)
        
    def init_position_plot(self):
        self.line_x, = self.ax_position.plot([], [], 'r-', label='X actual')
        self.line_y, = self.ax_position.plot([], [], 'g-', label='Y actual')
        self.line_z, = self.ax_position.plot([], [], 'b-', label='Z actual')
        
        self.line_x_d, = self.ax_position.plot([], [], 'r--', label='X desired')
        self.line_y_d, = self.ax_position.plot([], [], 'g--', label='Y desired')
        self.line_z_d, = self.ax_position.plot([], [], 'b--', label='Z desired')
        
        self.ax_position.set_title('End-Effector Position')
        self.ax_position.set_xlabel('Time (s)')
        self.ax_position.set_ylabel('Position (m)')
        self.ax_position.legend()
        self.ax_position.grid(True)
        
    def init_error_plot(self):
        self.line_x_err, = self.ax_error.plot([], [], 'r-', label='X error')
        self.line_y_err, = self.ax_error.plot([], [], 'g-', label='Y error')
        self.line_z_err, = self.ax_error.plot([], [], 'b-', label='Z error')
        self.ax_error.set_title('Position Error (Desired - Actual)')
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (m)')
        self.ax_error.legend()
        self.ax_error.grid(True)
        
    def force_callback(self, msg):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.fx_data.append(msg.wrench.force.x)
        self.fy_data.append(msg.wrench.force.y)
        self.fz_data.append(msg.wrench.force.z)
        
    def state_callback(self, msg):
        # Only process if we have time data (from force callback)
        if not self.time_data:
            return
            
        # Get current position from O_T_EE (4x4 transformation matrix)
        current_pos = np.array(msg.O_T_EE).reshape(4, 4)[:3, 3]
        self.x_data.append(current_pos[0])
        self.y_data.append(current_pos[1])
        self.z_data.append(current_pos[2])
        
        # Get desired position from O_T_EE_d
        desired_pos = np.array(msg.O_T_EE_d).reshape(4, 4)[:3, 3]
        if np.any(desired_pos):  # Only if not all zeros
            self.x_desired.append(desired_pos[0])
            self.y_desired.append(desired_pos[1])
            self.z_desired.append(desired_pos[2])
            
            # Calculate error
            self.x_error.append(desired_pos[0] - current_pos[0])
            self.y_error.append(desired_pos[1] - current_pos[1])
            self.z_error.append(desired_pos[2] - current_pos[2])
        
        self.update_plots()
    
    def update_plots(self):
        try:
            # Update force plot
            if self.time_data and self.fx_data:
                self.line_fx.set_data(self.time_data, self.fx_data)
                self.line_fy.set_data(self.time_data, self.fy_data)
                self.line_fz.set_data(self.time_data, self.fz_data)
                self.ax_force.relim()
                self.ax_force.autoscale_view()
            
            # Update position plot
            if self.time_data and self.x_data:
                x_len = int(len(self.x_data))
                time_points = list(self.time_data)[-x_len:]
                self.line_x.set_data(time_points, self.x_data)
                self.line_y.set_data(time_points, self.y_data)
                self.line_z.set_data(time_points, self.z_data)
                
                if self.x_desired:
                    desired_len = int(len(self.x_desired))
                    desired_time_points = list(self.time_data)[-desired_len:]
                    self.line_x_d.set_data(desired_time_points, self.x_desired)
                    self.line_y_d.set_data(desired_time_points, self.y_desired)
                    self.line_z_d.set_data(desired_time_points, self.z_desired)
                
                self.ax_position.relim()
                self.ax_position.autoscale_view()
            
            # Update error plot
            if self.time_data and self.x_error:
                error_len = int(len(self.x_error))
                error_time_points = list(self.time_data)[-error_len:]
                self.line_x_err.set_data(error_time_points, self.x_error)
                self.line_y_err.set_data(error_time_points, self.y_error)
                self.line_z_err.set_data(error_time_points, self.z_error)
                self.ax_error.relim()
                self.ax_error.autoscale_view()
            
            # Use plt.pause instead of direct canvas operations
            plt.pause(0.001)
            
        except Exception as e:
            rospy.logerr(f"Error in update_plots: {str(e)}")
    
    def clear_data(self, event):
        """Clear all data when button is clicked"""
        self.time_data.clear()
        self.fx_data.clear()
        self.fy_data.clear()
        self.fz_data.clear()
        self.x_data.clear()
        self.y_data.clear()
        self.z_data.clear()
        self.x_desired.clear()
        self.y_desired.clear()
        self.z_desired.clear()
        self.x_error.clear()
        self.y_error.clear()
        self.z_error.clear()
        
        # Reset plots
        self.update_plots()
    
    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    monitor = TestLogging()
    monitor.run()