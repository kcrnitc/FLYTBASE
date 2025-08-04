#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Standard library imports
import queue
import threading
import time
from datetime import datetime, timedelta

# Third-party imports
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D

# Local module imports
from conflict_checker import check_deconfliction
from utils import parse_time, euclidean_distance, interpolate_mission_path

class RealTimeSimulator:
    """Real-time drone movement simulator with 4D (3D+time) collision detection"""
    
    def __init__(self, primary_mission, simulated_flights, safety_distance=2.0, speed_multiplier=10, enable_4d=True):
        """
        Initialize the real-time simulator
        
        Args:
            primary_mission (dict): Primary drone mission
            simulated_flights (list): List of simulated flights
            safety_distance (float): Safety distance for collision detection
            speed_multiplier (int): Animation speed multiplier
            enable_4d (bool): Enable 4D visualization
        """
        self.primary_mission = primary_mission
        self.simulated_flights = simulated_flights
        self.safety_distance = safety_distance
        self.speed_multiplier = speed_multiplier
        self.enable_4d = enable_4d
        
        # Generate interpolated paths for all drones
        self.primary_path = interpolate_mission_path(primary_mission['waypoints'])
        self.simulated_paths = []
        for flight in simulated_flights:
            # Handle both drone_id and id field names
            drone_id = flight.get('drone_id', flight.get('id', 'Unknown_Drone'))
            self.simulated_paths.append({
                'id': drone_id,
                'path': interpolate_mission_path(flight['waypoints'])
            })
        
        # Get time boundaries
        self.start_time, self.end_time = self._get_time_boundaries()
        self.current_time = self.start_time
        
        # Collision tracking
        self.detected_collisions = []
        self.collision_queue = queue.Queue()
        
        # Animation properties
        self.fig = None
        self.ax = None
        self.animation = None
        self.is_running = False
        
        print(f"🎬 Real-time Simulator Initialized (4D Mode: {'ON' if enable_4d else 'OFF'})")
        print(f"   Time Range: {self.start_time.strftime('%H:%M:%S')} - {self.end_time.strftime('%H:%M:%S')}")
        print(f"   Total Duration: {(self.end_time - self.start_time).total_seconds():.1f} seconds")
        print(f"   Animation Speed: {speed_multiplier}x real-time")
        print(f"   Safety Distance: {safety_distance}m")
        if enable_4d:
            print(f"   4D Time Visualization: Current positions with time labels")
            print(f"   Complete flight paths always visible")
    
    def _get_time_boundaries(self):
        """Get the earliest start and latest end times from all missions"""
        all_times = []
        
        # Primary mission times
        for point in self.primary_path:
            timestamp = point['timestamp']
            if isinstance(timestamp, str):
                timestamp = parse_time(timestamp)
            all_times.append(timestamp)
        
        # Simulated flight times
        for sim_path in self.simulated_paths:
            for point in sim_path['path']:
                timestamp = point['timestamp']
                if isinstance(timestamp, str):
                    timestamp = parse_time(timestamp)
                all_times.append(timestamp)
        
        return min(all_times), max(all_times)
    
    def _initialize_legend(self):
        """Initialize all legend entries at the start to avoid duplicates"""
        # Add legend entries for all possible elements
        
        # Primary drone legend
        self.ax.scatter([], [], [], c='darkblue', s=300, marker='o', 
                       label='Primary Drone', alpha=1.0)
        
        # Simulated drones legends  
        colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink']
        dark_colors = ['darkred', 'darkgreen', 'darkorange', 'indigo', 'saddlebrown', 'deeppink']
        
        for i, sim_path in enumerate(self.simulated_paths):
            color = dark_colors[i % len(dark_colors)]
            self.ax.scatter([], [], [], c=color, s=250, marker='o', 
                           label=f"Drone {sim_path['id']}", alpha=1.0)
        
        # Collision legend
        self.ax.scatter([], [], [], c='red', s=500, marker='X', alpha=0.8, 
                       edgecolors='darkred', linewidth=3, label='COLLISION!')
    
    def get_drone_position_at_time(self, drone_path, target_time):
        """
        Get drone position at specific time using interpolation
        
        Args:
            drone_path (list): List of waypoints with timestamps
            target_time (datetime): Target time
            
        Returns:
            dict: Position data or None if drone not active at this time
        """
        # Find the waypoints that bracket the target time
        before_point = None
        after_point = None
        
        for i, point in enumerate(drone_path):
            # Parse timestamp if it's a string
            point_time = point['timestamp']
            if isinstance(point_time, str):
                point_time = parse_time(point_time)
            
            if point_time <= target_time:
                before_point = point
            elif point_time > target_time and after_point is None:
                after_point = point
                break
        
        # If target time is before or after mission, return None
        if before_point is None or after_point is None:
            return None
        
        # Parse timestamps for comparison and interpolation
        before_time = before_point['timestamp']
        if isinstance(before_time, str):
            before_time = parse_time(before_time)
        
        after_time = after_point['timestamp']
        if isinstance(after_time, str):
            after_time = parse_time(after_time)
        
        # If exact match, return the point
        if before_time == target_time:
            return before_point
        
        # Interpolate between points
        time_diff = (after_time - before_time).total_seconds()
        elapsed = (target_time - before_time).total_seconds()
        ratio = elapsed / time_diff if time_diff > 0 else 0
        
        # Handle both position array and x,y,z field formats
        def get_coords(point):
            if 'position' in point:
                return point['position'][0], point['position'][1], point['position'][2]
            else:
                return point['x'], point['y'], point['z']
        
        before_x, before_y, before_z = get_coords(before_point)
        after_x, after_y, after_z = get_coords(after_point)
        
        return {
            'x': before_x + (after_x - before_x) * ratio,
            'y': before_y + (after_y - before_y) * ratio,
            'z': before_z + (after_z - before_z) * ratio,
            'timestamp': target_time
        }
    
    def check_collisions_at_time(self, current_time):
        """
        Check for collisions at the current time
        
        Args:
            current_time (datetime): Current simulation time
            
        Returns:
            list: List of collision details
        """
        collisions = []
        
        # Get primary drone position
        primary_pos = self.get_drone_position_at_time(self.primary_path, current_time)
        if primary_pos is None:
            return collisions
        
        # Check against each simulated drone
        for sim_path in self.simulated_paths:
            sim_pos = self.get_drone_position_at_time(sim_path['path'], current_time)
            if sim_pos is None:
                continue
            
            # Calculate distance
            distance = euclidean_distance(primary_pos, sim_pos)
            
            # Check if collision occurs
            if distance < self.safety_distance:
                collision = {
                    'time': current_time,
                    'primary_pos': primary_pos,
                    'simulated_pos': sim_pos,
                    'simulated_drone': sim_path['id'],
                    'distance': distance,
                    'safety_violation': self.safety_distance - distance
                }
                collisions.append(collision)
        
        return collisions
    
    def animate_frame(self, frame):
        """Animation frame function with 4D visualization support"""
        # Calculate current simulation time
        total_duration = (self.end_time - self.start_time).total_seconds()
        time_step = total_duration / 200  # 200 frames total
        self.current_time = self.start_time + timedelta(seconds=frame * time_step)
        
        # Clear the plot
        self.ax.clear()
        
        # Set up the plot
        self.ax.set_xlabel('X Coordinate (m)')
        self.ax.set_ylabel('Y Coordinate (m)')
        self.ax.set_zlabel('Z Coordinate (m)')
        
        if self.enable_4d:
            title = f'4D Drone Simulation (3D Space + Time)\\nTime: {self.current_time.strftime("%H:%M:%S")}'
            subtitle = f'Bright spheres show current positions with time labels'
        else:
            title = f'Real-Time Drone Simulation\\nTime: {self.current_time.strftime("%H:%M:%S")}'
            subtitle = ''
        
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        if subtitle:
            self.ax.text2D(0.5, 0.95, subtitle, transform=self.ax.transAxes, 
                          ha='center', va='top', fontsize=10, style='italic')
        
        # Initialize legend entries for all elements (after clearing)
        self._initialize_legend()
        
        # Plot complete trajectories (always show full paths)
        self._plot_complete_trajectories(alpha=0.5)
        
        # Get and plot current drone positions
        active_drones = []
        
        # Primary drone
        primary_pos = self.get_drone_position_at_time(self.primary_path, self.current_time)
        if primary_pos:
            # In 4D mode, current position is highlighted as a bright sphere
            self.ax.scatter(primary_pos['x'], primary_pos['y'], primary_pos['z'], 
                           c='darkblue', s=300, marker='o', alpha=1.0)
            # Add dynamic time label that moves with the drone
            time_str = self.current_time.strftime('%H:%M:%S')
            self.ax.text(primary_pos['x'] + 1, primary_pos['y'] + 1, primary_pos['z'] + 1, 
                        time_str, fontsize=10, color='darkblue', fontweight='bold')
            active_drones.append(('Primary', primary_pos))
        
        # Simulated drones
        colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink']
        dark_colors = ['darkred', 'darkgreen', 'darkorange', 'indigo', 'saddlebrown', 'deeppink']
        for i, sim_path in enumerate(self.simulated_paths):
            sim_pos = self.get_drone_position_at_time(sim_path['path'], self.current_time)
            if sim_pos:
                color = dark_colors[i % len(dark_colors)]
                # In 4D mode, current position is highlighted as a bright sphere
                self.ax.scatter(sim_pos['x'], sim_pos['y'], sim_pos['z'], 
                               c=color, s=250, marker='o', alpha=1.0)
                # Add dynamic time label that moves with the drone
                time_str = self.current_time.strftime('%H:%M:%S')
                self.ax.text(sim_pos['x'] + 1, sim_pos['y'] + 1, sim_pos['z'] + 1, 
                            time_str, fontsize=9, color=color, fontweight='bold')
                active_drones.append((sim_path['id'], sim_pos))
        
        # Check for collisions
        collisions = self.check_collisions_at_time(self.current_time)
        
        # Handle new collisions with spatial-temporal deduplication
        for collision in collisions:
            # Check if this is a new conflict or continuation of existing one
            is_new_conflict = True
            
            for existing in self.detected_collisions:
                existing_collision = existing['collision']
                # Same drone pair
                if existing_collision['simulated_drone'] == collision['simulated_drone']:
                    # Check both time proximity and spatial proximity
                    time_diff = abs((collision['time'] - existing_collision['time']).total_seconds())
                    
                    # Calculate spatial distance between collision locations
                    from utils import euclidean_distance
                    spatial_distance = euclidean_distance(
                        collision['primary_pos'], existing_collision['primary_pos']
                    )
                    
                    # Group if within 2 minutes OR within 50m (same conflict zone)
                    if time_diff <= 120 or spatial_distance <= 50:
                        is_new_conflict = False
                        break
            
            if is_new_conflict:
                collision_id = f"{collision['simulated_drone']}_{len(self.detected_collisions)}"
                self.detected_collisions.append({
                    'id': collision_id,
                    'collision': collision
                })
                self.collision_queue.put(collision)
                print(f"🚨 COLLISION DETECTED at {collision['time'].strftime('%H:%M:%S')}!")
                print(f"   Primary vs {collision['simulated_drone']}")
                print(f"   Distance: {collision['distance']:.2f}m (Violation: {collision['safety_violation']:.2f}m)")
                print(f"   Primary Position: ({collision['primary_pos']['x']:.1f}, {collision['primary_pos']['y']:.1f}, {collision['primary_pos']['z']:.1f})")
                print(f"   {collision['simulated_drone']} Position: ({collision['simulated_pos']['x']:.1f}, {collision['simulated_pos']['y']:.1f}, {collision['simulated_pos']['z']:.1f})")
        
        # Highlight collision zones with cross marks for all colliding drones
        if collisions:
            for collision in collisions:
                # Mark primary drone collision position
                primary_pos = collision['primary_pos']
                self.ax.scatter(primary_pos['x'], primary_pos['y'], primary_pos['z'], 
                               c='red', s=600, marker='X', alpha=0.9, 
                               edgecolors='darkred', linewidth=4, label='Collision')
                
                # Mark simulated drone collision position
                sim_pos = collision['simulated_pos']
                self.ax.scatter(sim_pos['x'], sim_pos['y'], sim_pos['z'], 
                               c='red', s=600, marker='X', alpha=0.9, 
                               edgecolors='darkred', linewidth=4)
                
                # Add collision warning text
                mid_x = (primary_pos['x'] + sim_pos['x']) / 2
                mid_y = (primary_pos['y'] + sim_pos['y']) / 2
                mid_z = (primary_pos['z'] + sim_pos['z']) / 2 + 2
                self.ax.text(mid_x, mid_y, mid_z, 
                            f'COLLISION!\n{collision["distance"]:.1f}m', 
                            fontsize=12, color='red', fontweight='bold',
                            ha='center', va='center', 
                            bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.8))
        
        # Add information text
        info_text = f"Active Drones: {len(active_drones)}\\n"
        info_text += f"Collisions Detected: {len(self.detected_collisions)}\\n"
        info_text += f"Simulation Progress: {frame/200*100:.1f}%"
        
        if self.enable_4d:
            info_text += f"\\n4D Mode: Current positions with time labels"
            info_text += f"\\nComplete trajectories always visible"
        
        self.ax.text2D(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                      verticalalignment='top', fontsize=10,
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Set equal aspect ratio and reasonable limits
        self._set_equal_aspect_ratio()
        
        # Always show legend
        self.ax.legend(loc='upper right', fontsize=9)
    
    def _plot_complete_trajectories(self, alpha=0.5):
        """Plot complete trajectories for reference"""
        # Primary trajectory - THICKER LINE (linewidth=3)
        primary_x = [p['x'] for p in self.primary_path]
        primary_y = [p['y'] for p in self.primary_path]
        primary_z = [p['z'] for p in self.primary_path]
        self.ax.plot(primary_x, primary_y, primary_z, 'b-', alpha=alpha, linewidth=4)
        
        # Simulated trajectories - CONSISTENT THICKNESS (linewidth=2)
        colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink']
        for i, sim_path in enumerate(self.simulated_paths):
            color = colors[i % len(colors)]
            x_coords = [p['x'] for p in sim_path['path']]
            y_coords = [p['y'] for p in sim_path['path']]
            z_coords = [p['z'] for p in sim_path['path']]
            self.ax.plot(x_coords, y_coords, z_coords, color=color, alpha=alpha, linewidth=1.5)
    
    def _set_equal_aspect_ratio(self):
        """Set equal aspect ratio for 3D plot"""
        # Get all coordinates
        all_coords = []
        for p in self.primary_path:
            all_coords.append([p['x'], p['y'], p['z']])
        for sim_path in self.simulated_paths:
            for p in sim_path['path']:
                all_coords.append([p['x'], p['y'], p['z']])
        
        if all_coords:
            all_coords = np.array(all_coords)
            max_range = np.array([all_coords[:,0].max()-all_coords[:,0].min(),
                                 all_coords[:,1].max()-all_coords[:,1].min(),
                                 all_coords[:,2].max()-all_coords[:,2].min()]).max() / 2.0
            
            mid_x = (all_coords[:,0].max()+all_coords[:,0].min()) * 0.5
            mid_y = (all_coords[:,1].max()+all_coords[:,1].min()) * 0.5
            mid_z = (all_coords[:,2].max()+all_coords[:,2].min()) * 0.5
            
            self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    def start_simulation(self, save_path=None):
        """
        Start the real-time simulation
        
        Args:
            save_path (str): Optional path to save animation as GIF
        """
        mode_str = "4D Effects (3D+Time)" if self.enable_4d else "3D Real-Time"
        print(f"\\n🚀 Starting {mode_str} Simulation...")
        if self.enable_4d:
            print(f"🌈 4D Time Visualization:")
            print(f"   • Bright spheres showing current drone positions")
            print(f"   • Time labels moving with each drone")
            print(f"   • Complete flight paths always visible")
            print(f"   • Clean visualization - only current positions highlighted")
        print(f"Press Ctrl+C to stop the simulation\\n")
        
        # Create figure and 3D axis
        figsize = (14, 10) if self.enable_4d else (12, 8)
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Enhance the 3D view for better temporal visualization
        if self.enable_4d:
            self.ax.view_init(elev=25, azim=45)  # Better angle for 4D effects
        
        # Create animation
        self.animation = animation.FuncAnimation(
            self.fig, self.animate_frame, frames=200, interval=100, repeat=True
        )
        
        # Save animation if requested
        if save_path:
            print(f"💾 Saving animation to {save_path}...")
            self.animation.save(save_path, writer='pillow', fps=5)
            print(f"✅ Animation saved successfully!")
        
        # Show the animation
        plt.tight_layout()
        plt.show()
        
        return self.animation
    
    def get_collision_summary(self):
        """Get summary of detected collisions"""
        if not self.detected_collisions:
            return "✅ No collisions detected during simulation"
        
        summary = f"🚨 COLLISION SUMMARY ({len(self.detected_collisions)} total)\\n"
        summary += "=" * 50 + "\\n"
        
        for i, collision_data in enumerate(self.detected_collisions, 1):
            collision = collision_data['collision']
            summary += f"Collision {i}:\\n"
            summary += f"  Time: {collision['time'].strftime('%H:%M:%S')}\\n"
            summary += f"  Drones: Primary vs {collision['simulated_drone']}\\n"
            summary += f"  Distance: {collision['distance']:.2f}m\\n"
            summary += f"  Safety Violation: {collision['safety_violation']:.2f}m\\n"
            summary += f"  Primary Location: ({collision['primary_pos']['x']:.1f}, {collision['primary_pos']['y']:.1f}, {collision['primary_pos']['z']:.1f})\\n"
            summary += f"  {collision['simulated_drone']} Location: ({collision['simulated_pos']['x']:.1f}, {collision['simulated_pos']['y']:.1f}, {collision['simulated_pos']['z']:.1f})\\n\\n"
        
        return summary


def run_realtime_simulation(primary_mission, simulated_flights, safety_distance=2.0, speed_multiplier=10, save_animation=None, enable_4d=True):
    """
    Convenience function to run real-time simulation with 4D support
    """
    simulator = RealTimeSimulator(primary_mission, simulated_flights, safety_distance, speed_multiplier, enable_4d)
    
    try:
        animation = simulator.start_simulation(save_animation)
        
        input("\\nPress Enter to see collision summary...")
        
    except KeyboardInterrupt:
        print("\\n🛑 Simulation stopped by user")
    
    # Print collision summary
    print("\\n" + simulator.get_collision_summary())
    
    return simulator


if __name__ == "__main__":
    import sys
    import argparse
    import json
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Real-time drone simulation')
    parser.add_argument('primary_file', nargs='?', default='primary_drone_mission.json', help='Primary drone mission file')
    parser.add_argument('simulated_file', nargs='?', default='simulated_drones.json', help='Simulated drones file')
    parser.add_argument('--safety-distance', type=float, default=2.0, help='Safety distance for collision detection')
    parser.add_argument('--speed-multiplier', type=float, default=2.0, help='Animation speed multiplier')
    
    args = parser.parse_args()
    
    try:
        # Load primary mission from JSON file
        with open(args.primary_file, 'r') as f:
            primary = json.load(f)
        
        # Load simulated flights from JSON file
        with open(args.simulated_file, 'r') as f:
            simulated = json.load(f)
        
        print("🎬 Real-Time Simulation Demo")
        print("This will show drones moving in real-time with collision detection")
        
        simulator = run_realtime_simulation(primary, simulated, 
                                          safety_distance=args.safety_distance, 
                                          speed_multiplier=args.speed_multiplier)
        
    except Exception as e:
        print(f"Error running simulation: {e}")
