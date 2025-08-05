#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Standard library imports
import json
import os
import subprocess
import sys
from datetime import datetime, timedelta

# Third-party imports
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, TextBox
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog

# Local module imports
from conflict_checker import check_deconfliction


class PrimaryDronePlotter:

    
    def __init__(self):
        
        # MATPLOTLIB FIGURE INITIALIZATION
        # Create main plotting figure with optimal size for waypoint visualization
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        plt.subplots_adjust(bottom=0.15, right=0.75)  # Reserve space for controls
        
        # PRIMARY DRONE DATA STRUCTURES
        # Initialize all data structures for primary drone mission management
        self.primary_waypoints = []                    # List of waypoint dictionaries
        self.primary_start_time = '2025-08-04T10:00:00'  # Default mission start time
        self.primary_end_time = '2025-08-04T10:15:00'    # Default mission end time
        self.primary_points = []                       # Matplotlib plot points for visualization
        self.primary_line = None                       # Matplotlib line object for path
        
        # SIMULATED DRONES DATA STRUCTURES
        # Initialize structures for loading and managing simulated drone data
        self.other_drones = []                         # List of simulated drone missions
        
        # FILE MANAGEMENT CONFIGURATION
        # Fixed JSON file path for consistent primary drone mission storage
        self.primary_json_path = "primary_drone_mission.json"
        
        # INTERFACE SETUP SEQUENCE
        # Initialize all GUI components in proper order
        self.setup_plot()           # Configure plot appearance and coordinate system
        self.setup_controls()       # Create buttons, text fields, and control panels
        self.setup_event_handlers() # Bind mouse clicks and keyboard events
        
        # AUTO-LOAD SIMULATED DRONES
        # Automatically load simulated drones for immediate conflict detection capability
        self.auto_load_simulated_drones()
        
        print("Primary Drone Plotter initialized")
        print(f"Primary drone path will be saved to: {self.primary_json_path}")
    
    def setup_plot(self):
        """Setup the main plotting area"""
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')
        self.ax.set_title('Primary Drone Path Planner')
        self.ax.grid(True, alpha=0.3)
        
        # Add coordinate display
        self.coord_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                                     verticalalignment='top')
    
    def setup_controls(self):
        """Setup control buttons and inputs"""
        # Time controls
        start_time_ax = plt.axes([0.77, 0.85, 0.2, 0.04])
        self.start_time_box = TextBox(start_time_ax, 'Start Time: ', initial=self.primary_start_time)
        self.start_time_box.on_submit(self.update_start_time)
        
        end_time_ax = plt.axes([0.77, 0.80, 0.2, 0.04])
        self.end_time_box = TextBox(end_time_ax, 'End Time: ', initial=self.primary_end_time)
        self.end_time_box.on_submit(self.update_end_time)
        
        # Clear waypoints button
        clear_ax = plt.axes([0.77, 0.70, 0.2, 0.05])
        self.clear_btn = Button(clear_ax, 'Clear Primary Path')
        self.clear_btn.on_clicked(self.clear_primary_path)
        
        # Load other drones button
        load_others_ax = plt.axes([0.77, 0.60, 0.2, 0.05])
        self.load_others_btn = Button(load_others_ax, 'Load Other Drones')
        self.load_others_btn.on_clicked(self.load_other_drones)
        
        # Save primary path button
        save_ax = plt.axes([0.77, 0.50, 0.2, 0.05])
        self.save_btn = Button(save_ax, 'Save Primary Path')
        self.save_btn.on_clicked(self.save_primary_path)
        
        # Collision detection button
        collision_ax = plt.axes([0.77, 0.40, 0.2, 0.05])
        self.collision_btn = Button(collision_ax, 'Check Conflicts')
        self.collision_btn.on_clicked(self.check_conflicts)
        
        # Simulation button
        simulation_ax = plt.axes([0.77, 0.30, 0.2, 0.05])
        self.simulation_btn = Button(simulation_ax, 'Run Simulation')
        self.simulation_btn.on_clicked(self.run_simulation)
        
        # Instructions
        instructions = """
INSTRUCTIONS:
• Click on plot to add waypoints for primary drone
• Dialog will prompt for Z-coordinate (altitude)
• Right-click to remove last waypoint
• Set start/end times in ISO format
• Load other drones from JSON files
• Save overwrites primary_drone_mission.json
• Check Conflicts shows collision analysis
• Run Simulation shows real-time 4D simulation
• Primary drone path: BLUE
• Other drones: Various colors
        """
        self.fig.text(0.77, 0.02, instructions, fontsize=8, 
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
    
    def setup_event_handlers(self):
        """Setup mouse click handlers"""
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
    
    def auto_load_simulated_drones(self):
        """Automatically load simulated drones from simulated_drones.json if it exists"""
        try:
            simulated_file = "simulated_drones.json"
            if os.path.exists(simulated_file):
                with open(simulated_file, 'r') as f:
                    simulated_data = json.load(f)
                
                # Store data
                self.other_drones = simulated_data
                
                # Visualize other drones
                colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
                
                for i, drone in enumerate(self.other_drones):
                    color = colors[i % len(colors)]
                    waypoints = drone['waypoints']
                    
                    if waypoints:
                        # Plot waypoints - handle both position array and x,y format
                        x_coords = []
                        y_coords = []
                        
                        for wp in waypoints:
                            if 'position' in wp:
                                # New format: position array [x, y, z]
                                x_coords.append(wp['position'][0])
                                y_coords.append(wp['position'][1])
                            else:
                                # Old format: separate x, y fields
                                x_coords.append(wp['x'])
                                y_coords.append(wp['y'])
                        
                        # Plot points
                        self.ax.plot(x_coords, y_coords, 'o', color=color, markersize=6, alpha=0.7)
                        
                        # Plot connecting line
                        if len(waypoints) > 1:
                            self.ax.plot(x_coords, y_coords, '-', color=color, linewidth=1, alpha=0.5)
                        
                        # Add drone ID label at first waypoint
                        if waypoints:
                            # Handle both drone_id and id field names
                            drone_id = drone.get('drone_id', drone.get('id', f'Drone_{i+1}'))
                            self.ax.annotate(drone_id, (x_coords[0], y_coords[0]), 
                                           xytext=(-10, -10), textcoords='offset points', 
                                           fontsize=8, color=color, fontweight='bold')
                
                self.fig.canvas.draw()
                print(f"Loaded {len(self.other_drones)} other drones from {simulated_file}")
                
        except Exception as e:
            print(f"Error loading other drones: {e}")
            # Don't show error dialog during initialization
    
    def on_mouse_move(self, event):
        """Update coordinate display on mouse move"""
        if event.inaxes == self.ax:
            self.coord_text.set_text(f'X: {event.xdata:.1f}, Y: {event.ydata:.1f}')
            self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        """Handle mouse clicks for waypoint placement"""
        if event.inaxes != self.ax:
            return
        
        if event.button == 1:  # Left click - add waypoint
            # Create a temporary root window for the dialog
            root = tk.Tk()
            root.withdraw()  # Hide the main window
            
            # Prompt for Z coordinate
            z_value = simpledialog.askfloat(
                "Z Coordinate Input",
                f"Enter Z coordinate for waypoint at ({event.xdata:.2f}, {event.ydata:.2f}):\n\n" +
                "(Altitude in meters, default: 10.0)",
                initialvalue=10.0,
                minvalue=0.0,
                maxvalue=1000.0
            )
            
            root.destroy()  # Clean up the temporary window
            
            # If user cancelled or provided invalid input, use default
            if z_value is None:
                z_value = 10.0
            
            self.add_waypoint(event.xdata, event.ydata, z_value)
        elif event.button == 3:  # Right click - remove last waypoint
            self.remove_last_waypoint()
    
    def add_waypoint(self, x, y, z=10.0):
        """Add waypoint to primary drone path"""
        # Calculate timestamp based on position in sequence
        if self.primary_waypoints:
            # Distribute timestamps evenly between start and end times
            start_dt = datetime.fromisoformat(self.primary_start_time)
            end_dt = datetime.fromisoformat(self.primary_end_time)
            total_duration = (end_dt - start_dt).total_seconds()
            
            # Time for this waypoint (use whole seconds)
            waypoint_index = len(self.primary_waypoints)
            time_offset = int((waypoint_index + 1) * total_duration / (len(self.primary_waypoints) + 1))
            waypoint_time = start_dt + timedelta(seconds=time_offset)
            timestamp = waypoint_time.strftime('%Y-%m-%dT%H:%M:%S')
        else:
            timestamp = self.primary_start_time
        
        waypoint = {
            'x': round(x, 2),
            'y': round(y, 2),
            'z': z,
            'timestamp': timestamp
        }
        
        self.primary_waypoints.append(waypoint)
        
        # Add visual point
        point = self.ax.plot(x, y, 'o', color='blue', markersize=10)[0]
        self.primary_points.append(point)
        
        # Add waypoint number annotation
        self.ax.annotate(f'{len(self.primary_waypoints)}', (x, y), xytext=(5, 5), 
                        textcoords='offset points', fontsize=10, color='blue', fontweight='bold')
        
        # Update line connecting waypoints
        self.update_primary_line()
        self.fig.canvas.draw()
        
        print(f"Added waypoint {len(self.primary_waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f}) at {timestamp}")
    
    def remove_last_waypoint(self):
        """Remove the last waypoint from primary drone path"""
        if not self.primary_waypoints:
            return
        
        # Remove data
        removed_wp = self.primary_waypoints.pop()
        
        # Remove visual elements
        if self.primary_points:
            point = self.primary_points.pop()
            point.remove()
        
        # Clear all annotations and redraw them
        for txt in self.ax.texts[:]:
            if txt != self.coord_text:  # Keep coordinate display
                txt.remove()
        
        # Redraw annotations for remaining waypoints
        for i, wp in enumerate(self.primary_waypoints):
            self.ax.annotate(f'{i+1}', (wp['x'], wp['y']), xytext=(5, 5), 
                            textcoords='offset points', fontsize=10, color='blue', fontweight='bold')
        
        # Update line
        self.update_primary_line()
        self.fig.canvas.draw()
        
        print(f"Removed waypoint: ({removed_wp['x']}, {removed_wp['y']}, {removed_wp['z']})")
    
    def update_primary_line(self):
        """Update line connecting primary drone waypoints"""
        if self.primary_line:
            self.primary_line.remove()
            self.primary_line = None
        
        if len(self.primary_waypoints) > 1:
            x_coords = [wp['x'] for wp in self.primary_waypoints]
            y_coords = [wp['y'] for wp in self.primary_waypoints]
            self.primary_line = self.ax.plot(x_coords, y_coords, '-', color='blue', linewidth=2, alpha=0.7)[0]
    
    def update_start_time(self, text):
        """Update start time for primary drone"""
        try:
            datetime.fromisoformat(text)
            self.primary_start_time = text
            self.recalculate_timestamps()
            print(f"Primary drone start time updated to: {text}")
        except ValueError:
            messagebox.showerror("Invalid Time", "Use format: YYYY-MM-DDTHH:MM:SS")
            self.start_time_box.set_val(self.primary_start_time)
    
    def update_end_time(self, text):
        """Update end time for primary drone"""
        try:
            datetime.fromisoformat(text)
            self.primary_end_time = text
            self.recalculate_timestamps()
            print(f"Primary drone end time updated to: {text}")
        except ValueError:
            messagebox.showerror("Invalid Time", "Use format: YYYY-MM-DDTHH:MM:SS")
            self.end_time_box.set_val(self.primary_end_time)
    
    def recalculate_timestamps(self):
        """Recalculate timestamps for all waypoints based on start/end times"""
        if not self.primary_waypoints:
            return
        
        start_dt = datetime.fromisoformat(self.primary_start_time)
        end_dt = datetime.fromisoformat(self.primary_end_time)
        total_duration = (end_dt - start_dt).total_seconds()
        
        for i, waypoint in enumerate(self.primary_waypoints):
            if len(self.primary_waypoints) == 1:
                waypoint['timestamp'] = self.primary_start_time
            else:
                time_offset = int(i * total_duration / (len(self.primary_waypoints) - 1))
                waypoint_time = start_dt + timedelta(seconds=time_offset)
                waypoint['timestamp'] = waypoint_time.strftime('%Y-%m-%dT%H:%M:%S')
    
    def clear_primary_path(self, event=None):
        """Clear all primary drone waypoints"""
        # Clear data
        self.primary_waypoints.clear()
        
        # Clear visual elements
        for point in self.primary_points:
            point.remove()
        self.primary_points.clear()
        
        if self.primary_line:
            self.primary_line.remove()
            self.primary_line = None
        
        # Clear annotations (except coordinate display)
        for txt in self.ax.texts[:]:
            if txt != self.coord_text:
                txt.remove()
        
        self.fig.canvas.draw()
        print("Primary drone path cleared")
    
    def load_other_drones(self, event=None):
        """Load other drones from JSON files"""
        try:
            # Ask for simulated flights file
            filename = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Load Other Drones (Simulated Flights)"
            )
            
            if not filename:
                return
            
            with open(filename, 'r') as f:
                simulated_data = json.load(f)
            
            # Clear existing other drones visualization
            self.clear_other_drones_visualization()
            
            # Store data
            self.other_drones = simulated_data
            
            # Visualize other drones
            colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
            
            for i, drone in enumerate(self.other_drones):
                color = colors[i % len(colors)]
                waypoints = drone['waypoints']
                
                if waypoints:
                    # Plot waypoints - handle both position array and x,y format
                    x_coords = []
                    y_coords = []
                    
                    for wp in waypoints:
                        if 'position' in wp:
                            # New format: position array [x, y, z]
                            x_coords.append(wp['position'][0])
                            y_coords.append(wp['position'][1])
                        else:
                            # Old format: separate x, y fields
                            x_coords.append(wp['x'])
                            y_coords.append(wp['y'])
                    
                    # Plot points
                    self.ax.plot(x_coords, y_coords, 'o', color=color, markersize=6, alpha=0.7)
                    
                    # Plot connecting line
                    if len(waypoints) > 1:
                        self.ax.plot(x_coords, y_coords, '-', color=color, linewidth=1, alpha=0.5)
                    
                    # Add drone ID label at first waypoint
                    if waypoints:
                        # Handle both drone_id and id field names
                        drone_id = drone.get('drone_id', drone.get('id', f'Drone_{i+1}'))
                        self.ax.annotate(drone_id, (x_coords[0], y_coords[0]), 
                                       xytext=(-10, -10), textcoords='offset points', 
                                       fontsize=8, color=color, fontweight='bold')
            
            self.fig.canvas.draw()
            print(f"Loaded {len(self.other_drones)} other drones from {filename}")
            
        except Exception as e:
            print(f"Error loading other drones: {e}")
            messagebox.showerror("Load Error", f"Failed to load other drones: {e}")
    
    def clear_other_drones_visualization(self):
        """Clear visualization of other drones (keep primary drone intact)"""
        # This is a simplified approach - in a more complex implementation,
        # you'd track other drone visual elements separately
        # For now, we'll just redraw the primary drone after clearing
        pass
    
    def save_primary_path(self, event=None):
        """Save primary drone path to fixed JSON file (overwrites each time)"""
        try:
            if not self.primary_waypoints:
                messagebox.showwarning("No Data", "No waypoints to save for primary drone")
                return
            
            # Recalculate timestamps to ensure they're current
            self.recalculate_timestamps()
            
            # Create primary mission data
            primary_mission = {
                "waypoints": self.primary_waypoints,
                "time_window": {
                    "start": self.primary_start_time,
                    "end": self.primary_end_time
                }
            }
            
            # Save to fixed file path (overwrites each time)
            with open(self.primary_json_path, 'w') as f:
                json.dump(primary_mission, f, indent=2)
            
            print(f"✅ Primary drone path saved to: {self.primary_json_path}")
            print(f"   Waypoints: {len(self.primary_waypoints)}")
            print(f"   Time window: {self.primary_start_time} to {self.primary_end_time}")
            
            messagebox.showinfo("Save Complete", 
                              f"Primary drone path saved successfully!\n\n"
                              f"File: {self.primary_json_path}\n"
                              f"Waypoints: {len(self.primary_waypoints)}\n\n"
                              f"This file can be used by simulator and testing.")
            
        except Exception as e:
            print(f"ERROR saving primary path: {e}")
            messagebox.showerror("Save Error", f"Failed to save primary path: {e}")
    
    def check_conflicts(self, event=None):
        """Check for conflicts and display results in a new window"""
        try:
            if not self.primary_waypoints:
                messagebox.showwarning("No Data", "Please plot primary drone path first")
                return
            
            if not self.other_drones:
                messagebox.showwarning("No Other Drones", "Please load other drones first")
                return
            
            # Prepare primary mission data
            primary_mission = {
                "waypoints": self.primary_waypoints,
                "time_window": {
                    "start": self.primary_start_time,
                    "end": self.primary_end_time
                }
            }
            
            print("Running conflict detection...")
            
            # Run conflict detection with improved parameters
            safety_distance = 2.0  # Consistent safety distance for both detection and simulation
            result = check_deconfliction(primary_mission, self.other_drones, 
                                       safety_distance=safety_distance, time_threshold=10)
            
            # Create conflict details window
            self.show_conflict_window(result)
            
        except Exception as e:
            print(f"ERROR in conflict detection: {e}")
            messagebox.showerror("Conflict Detection Error", f"Failed to check conflicts: {e}")
    
    def show_conflict_window(self, conflict_result):
        """Show conflict detection results in a new window"""
        # Create new window
        conflict_window = tk.Toplevel()
        conflict_window.title("Conflict Detection Results")
        conflict_window.geometry("600x400")
        
        # Create scrollable text area
        frame = ttk.Frame(conflict_window)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        text_area = tk.Text(frame, wrap=tk.WORD, font=('Courier', 10))
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text_area.yview)
        text_area.configure(yscrollcommand=scrollbar.set)
        
        text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Prepare results text
        results_text = "CONFLICT DETECTION RESULTS\n"
        results_text += "=" * 50 + "\n\n"
        
        if conflict_result['status'] == 'conflict':
            results_text += f"⚠️  STATUS: CONFLICTS DETECTED\n"
            results_text += f"Number of conflicts: {len(conflict_result['details'])}\n\n"
            
            results_text += "CONFLICT DETAILS:\n"
            results_text += "-" * 30 + "\n"
            
            for i, conflict in enumerate(conflict_result['details'], 1):
                results_text += f"\nConflict #{i}:\n"
                results_text += f"  Location: ({conflict['location']['x']:.2f}, {conflict['location']['y']:.2f})\n"
                results_text += f"  Time: {conflict['time']}\n"
                results_text += f"  Distance: {conflict.get('distance', 'N/A'):.2f}m\n"
                if 'drone_id' in conflict:
                    results_text += f"  Conflicting Drone: {conflict['drone_id']}\n"
            
            results_text += "\n" + "=" * 50 + "\n"
            results_text += "RECOMMENDATION: Review and modify flight paths\n"
            results_text += "to maintain safe separation distances.\n"
            
        else:
            results_text += "✅ STATUS: NO CONFLICTS DETECTED\n\n"
            results_text += "All flight paths maintain safe separation.\n"
            results_text += "Mission is clear to proceed.\n\n"
            results_text += "SUMMARY:\n"
            results_text += "-" * 30 + "\n"
            results_text += f"Primary drone waypoints: {len(self.primary_waypoints)}\n"
            results_text += f"Other drones checked: {len(self.other_drones)}\n"
            results_text += f"Safety distance: 2.0m\n"
            results_text += f"Time threshold: 30s\n"
        
        # Insert text and make read-only
        text_area.insert(tk.END, results_text)
        text_area.config(state=tk.DISABLED)
        
        # Add close button
        close_btn = ttk.Button(conflict_window, text="Close", command=conflict_window.destroy)
        close_btn.pack(pady=10)
        
        print(f"Conflict detection complete: {conflict_result['status']}")
        if conflict_result['status'] == 'conflict':
            print(f"Found {len(conflict_result['details'])} conflicts")
    
    def run_simulation(self, event=None):
        """Run real-time simulation with primary drone and other drones"""
        try:
            if not self.primary_waypoints:
                messagebox.showwarning("No Data", "Please plot primary drone path first")
                return
            
            if not self.other_drones:
                messagebox.showwarning("No Other Drones", "Please load other drones first")
                return
            
            # Prepare primary mission data
            primary_mission = {
                "waypoints": self.primary_waypoints,
                "time_window": {
                    "start": self.primary_start_time,
                    "end": self.primary_end_time
                }
            }
            
            # Use the specific JSON files as requested
            primary_file = "primary_drone_mission.json"
            simulated_file = "simulated_drones.json"
            
            # Save primary mission to the specific file (overwrites)
            with open(primary_file, 'w') as f:
                json.dump(primary_mission, f, indent=2)
            
            # Note: Using existing test_enhanced_mission_simulated.json for other drones
            # (User should load this file using 'Load Other Drones' button first)
            
            print(f"Starting real-time simulation...")
            print(f"Primary drone: {len(self.primary_waypoints)} waypoints")
            print(f"Other drones: {len(self.other_drones)} drones")
            
            # Run conflict detection first with appropriate parameters
            safety_distance = 2.0  # Consistent safety distance for both detection and simulation
            result = check_deconfliction(primary_mission, self.other_drones, 
                                       safety_distance=safety_distance, time_threshold=10)
            
            if result['status'] == 'conflict':
                print(f"⚠️  WARNING: {len(result['details'])} conflicts detected")
                response = messagebox.askyesno(
                    "Conflicts Detected", 
                    f"Found {len(result['details'])} conflicts!\n\n"
                    "Do you want to proceed with simulation anyway?"
                )
                if not response:
                    return
                print(f"⚠️  Proceeding with simulation despite {len(result['details'])} conflicts")
            else:
                print("✅ No conflicts detected - simulation is safe")
            
            # Run real-time simulation with consistent parameters
            print("Launching real-time 4D simulation...")
            subprocess.run([
                sys.executable, 'realtime_simulator.py',
                primary_file,
                simulated_file,
                '--safety-distance', str(safety_distance)
            ])
            
        except Exception as e:
            print(f"ERROR in simulation: {e}")
            messagebox.showerror("Simulation Error", f"Failed to run simulation: {e}")
    
    def show(self):
        """Display the primary drone plotter"""
        plt.show()

def main():
    """Main function to run the primary drone plotter"""
    plotter = PrimaryDronePlotter()
    plotter.show()

if __name__ == "__main__":
    main()
