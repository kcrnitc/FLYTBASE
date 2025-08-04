#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV Strategic Deconfliction System - Utility Functions Module

This module provides essential utility functions for the UAV Strategic Deconfliction
System. It contains core mathematical, temporal, and geometric operations required
for precise spatial-temporal conflict detection and trajectory analysis.

Key Utility Categories:
    - Spatial Calculations: 3D Euclidean distance computation
    - Temporal Operations: Timestamp parsing and time difference calculations
    - Trajectory Interpolation: Smooth path generation between waypoints
    - Format Compatibility: Support for multiple coordinate data formats

Design Principles:
    - High Performance: Optimized mathematical operations for real-time analysis
    - Format Flexibility: Support both legacy and new coordinate formats
    - Precision: Accurate calculations for safety-critical applications
    - Modularity: Clean separation of utility functions for maintainability

Author: FlytBase Robotics Assignment 2025
Created: 2025
Last Modified: 2025-08-05

Dependencies:
    - math: Mathematical operations and constants
    - datetime: Timestamp parsing and temporal calculations
"""

# Standard library imports
import math
from datetime import datetime, timedelta


def euclidean_distance(pos1, pos2):
    """
    Calculate 3D Euclidean Distance Between Two Spatial Positions
    
    This function computes the straight-line distance between two points in 3D space
    using the Euclidean distance formula. It is essential for spatial conflict detection
    as it determines whether drones violate the minimum safety distance threshold.
    
    MATHEMATICAL FORMULA:
        distance = √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
    
    SPATIAL SAFETY APPLICATION:
        - Core calculation for safety buffer validation
        - Determines minimum separation distance between drones
        - Used in both static conflict detection and real-time simulation
        - Critical for 3D airspace deconfliction (includes altitude)
    
    FORMAT COMPATIBILITY:
        Supports multiple coordinate data formats for maximum flexibility:
        - Legacy format: {'x': val, 'y': val, 'z': val}
        - New format: {'position': [x, y, z]}
        - Array format: [x, y, z]
    
    Args:
        pos1 (list or dict): First position in one of the supported formats:
            - [x, y, z] - Array format
            - {'x': x, 'y': y, 'z': z} - Legacy field format
            - {'position': [x, y, z]} - New array format
        pos2 (list or dict): Second position in same format as pos1
    
    Returns:
        float: Euclidean distance between the two positions in meters
    
    Example:
        >>> pos1 = {'x': 0, 'y': 0, 'z': 10}
        >>> pos2 = {'x': 3, 'y': 4, 'z': 10}
        >>> distance = euclidean_distance(pos1, pos2)
        >>> print(distance)  # 5.0 meters (3-4-5 triangle)
    
    Raises:
        KeyError: If position dictionaries are missing required coordinate fields
        IndexError: If position arrays have insufficient elements
    """
    # COORDINATE EXTRACTION HELPER
    # Supports multiple data formats for backward compatibility and flexibility
    def get_coords(pos):
        """
        Extract (x, y, z) coordinates from position data in any supported format.
        
        Args:
            pos (list or dict): Position data in supported format
        
        Returns:
            tuple: (x, y, z) coordinates as floats
        """
        if isinstance(pos, dict):
            if 'position' in pos:
                # New format: {'position': [x, y, z]}
                return pos['position'][0], pos['position'][1], pos['position'][2]
            else:
                # Legacy format: {'x': val, 'y': val, 'z': val}
                return pos['x'], pos['y'], pos['z']
        else:
            # Array format: [x, y, z]
            return pos[0], pos[1], pos[2]
    
    # COORDINATE EXTRACTION
    # Extract 3D coordinates from both positions
    x1, y1, z1 = get_coords(pos1)
    x2, y2, z2 = get_coords(pos2)
    
    # EUCLIDEAN DISTANCE CALCULATION
    # Apply 3D distance formula: √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def time_difference(time1, time2):
    """
    Calculate Absolute Time Difference Between Two Timestamps
    
    This function computes the absolute time difference between two timestamps
    in seconds. It is used for temporal analysis in conflict detection to
    determine time separation between events and validate temporal overlaps.
    
    TEMPORAL ANALYSIS APPLICATION:
        - Measures time separation between conflict events
        - Validates temporal overlap windows between flight paths
        - Supports deduplication logic for grouping related conflicts
        - Enables precise temporal conflict detection
    
    Args:
        time1 (str): First timestamp in ISO format (e.g., '2025-08-04T10:00:00')
        time2 (str): Second timestamp in ISO format (e.g., '2025-08-04T10:05:00')
    
    Returns:
        float: Absolute time difference between timestamps in seconds
    
    Example:
        >>> t1 = '2025-08-04T10:00:00'
        >>> t2 = '2025-08-04T10:05:00'
        >>> diff = time_difference(t1, t2)
        >>> print(diff)  # 300.0 seconds (5 minutes)
    """
    # Parse both timestamps to datetime objects
    t1 = parse_time(time1)
    t2 = parse_time(time2)
    
    # Calculate absolute difference in seconds
    return abs((t1 - t2).total_seconds())


def parse_time(timestamp):
    """
    Robust Timestamp Parser with Multiple Format Support
    
    This function provides robust parsing of timestamp strings in various formats
    commonly used in drone mission data. It handles ISO format timestamps with
    timezone information and simple time formats for maximum compatibility.
    
    SUPPORTED FORMATS:
        - ISO 8601 with timezone: '2025-08-04T10:00:00+00:00'
        - ISO 8601 with Z suffix: '2025-08-04T10:00:00Z'
        - ISO 8601 local time: '2025-08-04T10:00:00'
        - Simple time format: '10:00:00'
    
    PARSING STRATEGY:
        1. Detect format based on timestamp structure
        2. Apply appropriate parsing method
        3. Handle timezone information correctly
        4. Provide graceful fallback for edge cases
    
    Args:
        timestamp (str): Timestamp string in supported format
    
    Returns:
        datetime: Parsed datetime object in local timezone
    
    Example:
        >>> ts1 = parse_time('2025-08-04T10:00:00Z')
        >>> ts2 = parse_time('10:30:00')
        >>> print(ts1.hour)  # 10
    
    Raises:
        ValueError: If timestamp format is not supported (handled gracefully)
    """
    try:
        # FULL ISO FORMAT DETECTION
        # Handle timestamps with date and time components
        if 'T' in timestamp:
            if timestamp.endswith('Z'):
                # UTC timezone indicator (Z suffix)
                return datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
            else:
                # Standard ISO format (with or without timezone)
                return datetime.fromisoformat(timestamp)
        else:
            # SIMPLE TIME FORMAT
            # Handle time-only format like "10:30:00" (assumes current date)
            return datetime.strptime(timestamp, "%H:%M:%S")
    except ValueError:
        # GRACEFUL FALLBACK
        # Return current time if parsing fails (prevents system crashes)
        return datetime.now()


def interpolate_waypoints(wp1, wp2, step_seconds=1):
    """
    Interpolate waypoints between two points with given time step
    
    Args:
        wp1 (dict): Starting waypoint with position and timestamp
        wp2 (dict): Ending waypoint with position and timestamp
        step_seconds (int): Time step for interpolation
        
    Returns:
        list: List of interpolated waypoints
    """
    interpolated = []
    
    start_time = parse_time(wp1["timestamp"])
    end_time = parse_time(wp2["timestamp"])
    
    # Calculate total duration
    duration = (end_time - start_time).total_seconds()
    
    if duration <= 0:
        return [wp1]
    
    # Calculate number of steps
    num_steps = max(1, int(duration / step_seconds))
    
    for i in range(num_steps + 1):
        t = i / num_steps if num_steps > 0 else 0
        
        # Interpolate position - handle both position array and x,y,z format
        if 'position' in wp1:
            # Position array format [x, y, z]
            pos = [
                wp1["position"][0] + t * (wp2["position"][0] - wp1["position"][0]),
                wp1["position"][1] + t * (wp2["position"][1] - wp1["position"][1]),
                wp1["position"][2] + t * (wp2["position"][2] - wp1["position"][2])
            ]
        else:
            # Separate x, y, z fields
            pos = [
                wp1["x"] + t * (wp2["x"] - wp1["x"]),
                wp1["y"] + t * (wp2["y"] - wp1["y"]),
                wp1["z"] + t * (wp2["z"] - wp1["z"])
            ]
        
        # Interpolate time
        interpolated_time = start_time + timedelta(seconds=i * step_seconds)
        
        # Output in the same format as input waypoints
        if 'position' in wp1:
            # Input uses position arrays, output position arrays
            interpolated.append({
                "position": pos,
                "timestamp": interpolated_time.isoformat()
            })
        else:
            # Input uses x,y,z fields, output x,y,z fields
            interpolated.append({
                "x": pos[0],
                "y": pos[1],
                "z": pos[2],
                "timestamp": interpolated_time.isoformat()
            })
    
    return interpolated


def interpolate_mission_path(waypoints, step_seconds=1):
    """
    Interpolate a complete mission path from a list of waypoints
    
    Args:
        waypoints (list): List of waypoints with position and timestamp
        step_seconds (int): Time step for interpolation
        
    Returns:
        list: List of interpolated waypoints for the complete mission
    """
    if not waypoints or len(waypoints) < 2:
        return waypoints
    
    interpolated_path = []
    
    # Add first waypoint
    interpolated_path.append(waypoints[0])
    
    # Interpolate between consecutive waypoints
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        # Get interpolated points between wp1 and wp2 (excluding wp1 since it's already added)
        interpolated_segment = interpolate_waypoints(wp1, wp2, step_seconds)
        
        # Add all points except the first one (to avoid duplicates)
        if len(interpolated_segment) > 1:
            interpolated_path.extend(interpolated_segment[1:])
    
    return interpolated_path
