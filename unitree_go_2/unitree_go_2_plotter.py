import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

from ark.tools.log import log

class UnitreeGo2Plotter:
    def __init__(self):
        """Set up plot for Unitree Go 2 data. Plot should be a 3,1 subplot showing the odometry estimates (v_x, v_y, w)."""
        self.max_points = 1000  # Maximum number of points to keep in history
        self.created_plot = False

    def create_plot(self):
        # Enable interactive mode for live updating
        plt.ion()

        # Create figure and subplots
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8))
        self.fig.suptitle('Odometry Data Visualization', fontsize=16)

        # Set up subplot titles and labels
        labels = ['Linear X (v_x)', 'Linear Y (v_y)', 'Angular (w)']

        for i in range(3):
            # Odometry plots
            self.axes[i].set_title(labels[i])

            if i != 2:
                self.axes[i].set_ylabel('Velocity (m/s)')
            else:
                self.axes[i].set_ylabel('Velocity (rad/s)')
            self.axes[i].grid(True, alpha=0.3)

        # Data storage for plotting (using deque for efficient append/pop operations)
        self.time_data = deque(maxlen=self.max_points)
        self.odometry_data = [deque(maxlen=self.max_points) for _ in range(3)]
        self.unitree_odometry_data = [deque(maxlen=self.max_points) for _ in range(3)]

        # Initialize line objects for each plot
        self.lines = []
        self.unitree_lines = []

        colors = ['red', 'green', 'blue']
        for i in range(3):
            # Create line objects for regular odometry (solid lines)
            line, = self.axes[i].plot([], [], color=colors[i], linewidth=2, label='Odometry')
            self.lines.append(line)

            # Create line objects for unitree odometry (dashed lines)
            unitree_line, = self.axes[i].plot([], [], color=colors[i], linewidth=3,
                                              linestyle='--', alpha=0.8, label='Unitree Odometry')
            self.unitree_lines.append(unitree_line)

        # Add legends to each subplot
        for i in range(3):
            self.axes[i].legend(loc='upper right')

        # Initialize start time
        self.start_time = time.time()

        # Adjust layout to prevent overlap
        plt.tight_layout()

        # Show the plot window
        plt.show(block=False)
        self.created_plot = True

    def update(self, data):
        """Update the plots with new odometry data"""

        if not self.created_plot:
            self.create_plot()

        odometry = data["odometry"]

        # Calculate current time relative to start
        current_time = time.time() - self.start_time

        # Add new data points
        self.time_data.append(current_time)
        self.odometry_data[0].append(odometry["v_x"])
        self.odometry_data[1].append(odometry["v_y"])
        self.odometry_data[2].append(odometry["w"])

        # Check if unitree_odometry exists and add data
        if "unitree_odometry" in data:
            unitree_odometry = data["unitree_odometry"]
            self.unitree_odometry_data[0].append(unitree_odometry["v_x"])
            self.unitree_odometry_data[1].append(unitree_odometry["v_y"])
            self.unitree_odometry_data[2].append(unitree_odometry["w"])
        else:
            # If no unitree_odometry data, append None to maintain alignment
            for i in range(3):
                self.unitree_odometry_data[i].append(None)

        # Convert deques to lists for plotting
        time_list = list(self.time_data)

        # Update each subplot
        for i in range(3):
            odometry_list = list(self.odometry_data[i])

            # Update regular odometry plot
            self.lines[i].set_data(time_list, odometry_list)

            # Update unitree odometry plot if data exists
            unitree_odometry_list = list(self.unitree_odometry_data[i])

            # Filter out None values for plotting unitree odometry
            unitree_time_filtered = []
            unitree_data_filtered = []
            for t, val in zip(time_list, unitree_odometry_list):
                if val is not None:
                    unitree_time_filtered.append(t)
                    unitree_data_filtered.append(val)

            self.unitree_lines[i].set_data(unitree_time_filtered, unitree_data_filtered)

            # Update axis limits
            self.axes[i].relim()
            self.axes[i].autoscale_view()

        # Redraw the plots
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def stop(self):
        """
        Stop the plotter
        """
        log.ok("Stopping plotting...")

        # Close matplotlib window if it exists
        if hasattr(self, 'fig'):
            plt.close(self.fig)