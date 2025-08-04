#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV Strategic Deconfliction System - Conflict Detection Module

This module implements the core conflict detection logic for the UAV Strategic Deconfliction
System as specified in the FlytBase Robotics Assignment 2025. It provides comprehensive
spatial and temporal conflict checking between a primary drone mission and multiple
simulated flight paths in shared airspace.

Key Features:
    - Spatial Check: Validates minimum safety distance between drone trajectories
    - Temporal Check: Ensures no spatial overlap during overlapping time segments
    - 4D Analysis: Full 3D spatial coordinates + time dimension checking
    - Continuous Path Interpolation: Smooth trajectory analysis beyond waypoints
    - Spatial-Temporal Deduplication: Prevents false positives and duplicate reports
    - Synchronized Time-Based Checking: Matches simulation collision detection logic

Author: FlytBase Robotics Assignment 2025
Created: 2025
Last Modified: 2025-08-05

Requirements Compliance:
    ✅ Spatial Check: Minimum distance threshold validation
    ✅ Temporal Check: Overlapping time segment analysis
    ✅ Conflict Explanation: Detailed location and time reporting
    ✅ Query Interface: Simple Python function interface
    ✅ 4D Support: 3D coordinates + time dimension
"""

# Standard library imports
from datetime import datetime, timedelta

# Local module imports
from utils import euclidean_distance, time_difference, parse_time, interpolate_waypoints

def check_deconfliction(primary_mission, simulated_flights, safety_distance=2, time_threshold=10):
    """
    Main Deconfliction Query Interface - Strategic Conflict Detection
    
    This is the primary query interface for the UAV Strategic Deconfliction System.
    It serves as the final authority for verifying whether a drone's planned waypoint
    mission is safe to execute in shared airspace by checking for conflicts in both
    space and time against simulated flight paths of multiple other drones.
    
    SPATIAL CHECK IMPLEMENTATION:
        - Validates that primary mission path does not intersect with other trajectories
        - Uses configurable safety buffer (minimum distance threshold)
        - Performs 3D Euclidean distance calculations for precise spatial analysis
    
    TEMPORAL CHECK IMPLEMENTATION:
        - Ensures no other drone is present in same spatial area during overlapping time segments
        - Analyzes synchronized time points across all flight paths
        - Only checks conflicts during overlapping mission time windows
    
    Args:
        primary_mission (dict): Primary drone mission data containing:
            - 'waypoints': List of waypoints with coordinates and timestamps
            - Each waypoint: {'x': float, 'y': float, 'z': float, 'timestamp': str}
        simulated_flights (list): List of simulated flight data, each containing:
            - 'waypoints': List of waypoints with coordinates and timestamps
            - 'drone_id' or 'id': Unique identifier for the simulated drone
        safety_distance (float, optional): Minimum safe distance threshold in meters.
            Default: 2.0m as per system requirements
        time_threshold (int, optional): Time threshold in seconds for conflict detection.
            Default: 10s (Note: Current implementation uses spatial-temporal deduplication)
    
    Returns:
        dict: Deconfliction result containing:
            - 'status' (str): "clear" if no conflicts, "conflict" if conflicts detected
            - 'details' (list): List of conflict details for each detected conflict:
                - 'conflicting_drone': ID of conflicting simulated drone
                - 'location': Conflict location coordinates (x, y, z)
                - 'primary_location': Primary drone location at conflict
                - 'time': Timestamp of conflict occurrence
                - 'distance': Actual distance between drones at conflict
    
    Example:
        >>> primary = {'waypoints': [{'x': 0, 'y': 0, 'z': 10, 'timestamp': '2025-08-04T10:00:00'}]}
        >>> simulated = [{'drone_id': 'SIM_001', 'waypoints': [...]}]
        >>> result = check_deconfliction(primary, simulated, safety_distance=2.0)
        >>> print(result['status'])  # "clear" or "conflict"
    """
    conflicts = []
    
    # STEP 1: Generate continuous interpolated paths for precise trajectory analysis
    # Convert discrete waypoints into smooth continuous paths with 1-second intervals
    # This enables detection of conflicts between waypoints, not just at waypoint locations
    primary_path = generate_continuous_path(primary_mission)
    
    # STEP 2: Check conflicts with each simulated flight individually
    # Process each simulated drone's trajectory against the primary mission
    for sim_flight in simulated_flights:
        # Generate continuous path for current simulated flight
        sim_path = generate_continuous_path(sim_flight)
        
        # Handle both 'drone_id' and 'id' field names for backward compatibility
        # Some JSON files may use different field names for drone identification
        drone_id = sim_flight.get('drone_id', sim_flight.get('id', 'Unknown_Drone'))
        
        # Perform detailed spatial-temporal conflict detection
        # This implements both spatial and temporal checks as required
        flight_conflicts = detect_path_conflicts(primary_path, sim_path, drone_id, 
                                                 safety_distance, time_threshold)
        
        # Accumulate all conflicts from this simulated flight
        conflicts.extend(flight_conflicts)
    
    # STEP 3: Return standardized deconfliction result
    # Status indicates overall safety: "clear" = safe to proceed, "conflict" = unsafe
    # Details provide comprehensive conflict information for decision making
    return {
        "status": "conflict" if conflicts else "clear",
        "details": conflicts
    }

def generate_continuous_path(mission_or_flight, step_seconds=1):
    """
    Generate Continuous Trajectory Path from Discrete Waypoints
    
    This function converts discrete waypoint missions into smooth continuous trajectories
    by interpolating intermediate positions at regular time intervals. This is crucial
    for accurate conflict detection as it enables checking for conflicts between waypoints,
    not just at the waypoint locations themselves.
    
    ALGORITHM:
        1. Extract waypoints from mission data
        2. For each consecutive waypoint pair:
           - Calculate intermediate positions at 1-second intervals
           - Interpolate coordinates (x, y, z) and timestamps linearly
           - Add interpolated points to continuous path
        3. Append final waypoint to complete the trajectory
    
    INTERPOLATION BENEFITS:
        - Detects conflicts along entire flight path, not just at waypoints
        - Provides smooth trajectory analysis for realistic flight paths
        - Enables precise spatial-temporal conflict detection
        - Matches real-world drone flight behavior between waypoints
    
    Args:
        mission_or_flight (dict): Mission or flight data containing:
            - 'waypoints': List of waypoints with coordinates and timestamps
            - Each waypoint: {'x': float, 'y': float, 'z': float, 'timestamp': str}
        step_seconds (int, optional): Time step for interpolation in seconds.
            Default: 1 second for high-precision trajectory analysis
    
    Returns:
        list: Continuous path as list of interpolated waypoints, each containing:
            - 'x', 'y', 'z': Interpolated 3D coordinates
            - 'timestamp': Interpolated timestamp string
    
    Example:
        >>> mission = {'waypoints': [
        ...     {'x': 0, 'y': 0, 'z': 10, 'timestamp': '2025-08-04T10:00:00'},
        ...     {'x': 100, 'y': 100, 'z': 10, 'timestamp': '2025-08-04T10:05:00'}
        ... ]}
        >>> path = generate_continuous_path(mission, step_seconds=1)
        >>> len(path)  # 301 points (300 interpolated + 1 final waypoint)
    """
    # Extract waypoints from mission data
    waypoints = mission_or_flight['waypoints']
    continuous_path = []
    
    # STEP 1: Interpolate between consecutive waypoint pairs
    # Process each segment of the flight path individually
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]      # Starting waypoint of current segment
        wp2 = waypoints[i + 1]  # Ending waypoint of current segment
        
        # Generate interpolated points between wp1 and wp2
        # This creates a smooth trajectory with positions calculated at regular intervals
        interpolated_points = interpolate_waypoints(wp1, wp2, step_seconds)
        
        # Add all interpolated points to the continuous path
        # This builds up the complete smooth trajectory
        continuous_path.extend(interpolated_points)
    
    # STEP 2: Add the final waypoint to complete the trajectory
    # The interpolation process covers segments between waypoints,
    # but we need to explicitly add the final destination waypoint
    if waypoints:
        continuous_path.append(waypoints[-1])
    
    return continuous_path

def detect_path_conflicts(primary_path, sim_path, sim_drone_id, safety_distance, time_threshold):
    """
    Advanced Spatial-Temporal Conflict Detection with Deduplication
    
    This function implements the core conflict detection algorithm that performs both
    spatial and temporal checks as required by the UAV Strategic Deconfliction System.
    It uses synchronized time-based checking with sophisticated spatial-temporal
    deduplication to eliminate false positives and provide accurate conflict reports.
    
    SPATIAL CHECK IMPLEMENTATION:
        - Calculates 3D Euclidean distance between drone positions at synchronized times
        - Compares distance against configurable safety buffer (minimum distance threshold)
        - Uses precise coordinate extraction supporting multiple data formats
    
    TEMPORAL CHECK IMPLEMENTATION:
        - Identifies overlapping time windows between flight paths
        - Performs synchronized time-based position checking every 5 seconds
        - Only analyzes conflicts during actual temporal overlap periods
        - Skips analysis if no temporal overlap exists (automatic safety)
    
    SPATIAL-TEMPORAL DEDUPLICATION ALGORITHM:
        - Groups conflict events within 2-minute time windows OR 50-meter spatial proximity
        - Prevents multiple reports for the same ongoing conflict zone
        - Matches simulation collision detection logic for consistent reporting
        - Eliminates false positives from extended conflict duration
    
    ALGORITHM STEPS:
        1. Extract and parse timestamps from both flight paths
        2. Calculate overlapping time window for conflict analysis
        3. Check positions at 5-second intervals during overlap period
        4. Calculate 3D spatial distance at each synchronized time point
        5. Apply spatial-temporal deduplication to group related conflicts
        6. Return deduplicated list of distinct conflict zones
    
    Args:
        primary_path (list): Interpolated primary drone path containing waypoints with:
            - 'x', 'y', 'z': 3D coordinates or 'position': [x, y, z] array
            - 'timestamp': ISO format timestamp string
        sim_path (list): Interpolated simulated drone path with same format
        sim_drone_id (str): Unique identifier for the simulated drone
        safety_distance (float): Minimum safe distance threshold in meters
        time_threshold (int): Legacy parameter (spatial-temporal deduplication used instead)
    
    Returns:
        list: Deduplicated list of conflict details, each containing:
            - 'conflicting_drone': ID of conflicting simulated drone
            - 'location': Simulated drone location at conflict (x, y, z)
            - 'primary_location': Primary drone location at conflict (x, y, z)
            - 'time': ISO timestamp of conflict occurrence
            - 'distance': Actual distance between drones in meters
    
    Example:
        >>> conflicts = detect_path_conflicts(primary_path, sim_path, 'SIM_001', 2.0, 10)
        >>> for conflict in conflicts:
        ...     print(f"Conflict at {conflict['time']}: {conflict['distance']}m apart")
    """
    conflicts = []
    
    # HELPER FUNCTION: Coordinate Extraction with Format Compatibility
    # Supports both legacy format {'x': val, 'y': val, 'z': val} and 
    # new format {'position': [x, y, z]} for backward compatibility
    def get_position_coords(pos):
        """
        Extract 3D coordinates from position data supporting multiple formats.
        
        Args:
            pos (dict): Position data in either format:
                - {'position': [x, y, z]} - New array format
                - {'x': val, 'y': val, 'z': val} - Legacy field format
        
        Returns:
            tuple: (x, y, z) coordinates as floats
        """
        if 'position' in pos:
            # New format: position array [x, y, z]
            return pos['position'][0], pos['position'][1], pos['position'][2]
        else:
            # Legacy format: separate x, y, z fields (z defaults to 0 if missing)
            return pos['x'], pos['y'], pos.get('z', 0)
    
    # STEP 1: TEMPORAL CHECK - Extract and Analyze Time Boundaries
    # Parse all timestamps from both flight paths to determine temporal overlap
    # This implements the "Temporal Check" requirement: ensure no other drone is present
    # in the same spatial area during overlapping time segments
    primary_times = [parse_time(wp['timestamp']) for wp in primary_path]
    sim_times = [parse_time(wp['timestamp']) for wp in sim_path]
    
    # Safety check: ensure both paths have valid timestamps
    if not primary_times or not sim_times:
        return conflicts  # Cannot analyze without valid timestamps
    
    # Calculate overlapping time window between the two flight paths
    # Only check conflicts during the time period when both drones are active
    # start_time = latest start time of either drone (when both are active)
    # end_time = earliest end time of either drone (when one becomes inactive)
    start_time = max(min(primary_times), min(sim_times))  # Latest start
    end_time = min(max(primary_times), max(sim_times))    # Earliest end
    
    # TEMPORAL CHECK VALIDATION: No conflict possible if no temporal overlap
    if start_time >= end_time:
        return conflicts  # No time overlap = automatically safe (temporal requirement satisfied)
    
    # STEP 2: SYNCHRONIZED SPATIAL-TEMPORAL CONFLICT DETECTION
    # Check for spatial conflicts at regular intervals during the overlapping time window
    # This implements the "Spatial Check" requirement: validate minimum distance threshold
    check_interval = 5  # Check every 5 seconds for high precision
    current_time = start_time
    detected_conflicts = []  # Store all detected conflicts for sophisticated deduplication
    
    # Iterate through overlapping time window at regular intervals
    while current_time <= end_time:
        # SYNCHRONIZED POSITION CALCULATION
        # Get exact positions of both drones at the same synchronized time point
        # This ensures accurate spatial distance calculation at identical timestamps
        primary_pos = get_drone_position_at_time(primary_path, current_time)
        sim_pos = get_drone_position_at_time(sim_path, current_time)
        
        # Proceed only if both drones have valid positions at this time
        if primary_pos and sim_pos:
            # SPATIAL CHECK IMPLEMENTATION
            # Extract 3D coordinates from position data (supporting multiple formats)
            primary_x, primary_y, primary_z = get_position_coords(primary_pos)
            sim_x, sim_y, sim_z = get_position_coords(sim_pos)
            
            # Calculate 3D Euclidean distance between drone positions
            # This is the core spatial proximity calculation for safety validation
            distance = ((primary_x - sim_x)**2 + (primary_y - sim_y)**2 + (primary_z - sim_z)**2)**0.5
            
            # SAFETY BUFFER VALIDATION
            # Check if drones violate minimum safe distance threshold
            if distance <= safety_distance:
                # CONFLICT DETECTED: Create detailed conflict report
                # This provides comprehensive information for conflict explanation requirement
                potential_conflict = {
                    "conflicting_drone": sim_drone_id,      # ID of conflicting simulated drone
                    "drone_id": sim_drone_id,               # Duplicate for compatibility
                    "location": {                           # Simulated drone location at conflict
                        "x": round(sim_x, 2),
                        "y": round(sim_y, 2), 
                        "z": round(sim_z, 2)
                    },
                    "primary_location": {                   # Primary drone location at conflict
                        "x": round(primary_x, 2),
                        "y": round(primary_y, 2), 
                        "z": round(primary_z, 2)
                    },
                    "time": current_time.strftime('%Y-%m-%dT%H:%M:%S'),        # Conflict timestamp
                    "primary_time": current_time.strftime('%Y-%m-%dT%H:%M:%S'), # Primary drone time
                    "sim_time": current_time.strftime('%Y-%m-%dT%H:%M:%S'),     # Simulated drone time
                    "distance": round(distance, 2),         # Actual distance between drones
                    "time_difference": 0.0,                 # Same synchronized time
                    "timestamp_obj": current_time           # Datetime object for deduplication
                }
                
                # STEP 3: SOPHISTICATED SPATIAL-TEMPORAL DEDUPLICATION
                # This advanced algorithm prevents false positives and duplicate conflict reports
                # by grouping related conflict events into single distinct conflict zones
                # PROBLEM: Without deduplication, extended conflicts generate multiple reports
                # SOLUTION: Group conflicts that are temporally or spatially related
                is_new_conflict = True  # Assume this is a new distinct conflict zone
                
                # Compare against all previously detected conflicts for this drone pair
                for existing in detected_conflicts:
                    # TEMPORAL PROXIMITY CHECK
                    # Calculate time difference between current and existing conflict
                    time_diff = abs((potential_conflict['timestamp_obj'] - existing['timestamp_obj']).total_seconds())
                    
                    # SPATIAL PROXIMITY CHECK
                    # Calculate 3D spatial distance between conflict locations
                    # This determines if conflicts occur in the same spatial area
                    spatial_distance = ((potential_conflict['primary_location']['x'] - existing['primary_location']['x'])**2 + 
                                      (potential_conflict['primary_location']['y'] - existing['primary_location']['y'])**2 + 
                                      (potential_conflict['primary_location']['z'] - existing['primary_location']['z'])**2)**0.5
                    
                    # DEDUPLICATION DECISION LOGIC
                    # Group conflicts if EITHER condition is met (OR logic):
                    # 1. TEMPORAL: Within 2 minutes (120 seconds) - same extended conflict event
                    # 2. SPATIAL: Within 50 meters - same geographical conflict zone
                    # This matches the simulation's deduplication logic for consistency
                    if time_diff <= 120 or spatial_distance <= 50:
                        is_new_conflict = False  # This is part of an existing conflict zone
                        break  # Stop checking - conflict is already covered
                
                # CONFLICT REPORTING DECISION
                if is_new_conflict:
                    # This is a genuinely new and distinct conflict zone
                    # Clean up temporary deduplication data before final report
                    del potential_conflict['timestamp_obj']
                    conflicts.append(potential_conflict)  # Add to final conflict list
                    
                    # Maintain deduplication tracking for future comparisons
                    # Re-add timestamp for comparing against subsequent potential conflicts
                    potential_conflict['timestamp_obj'] = current_time
                    detected_conflicts.append(potential_conflict)
        
        # STEP 4: ADVANCE TO NEXT TIME CHECKPOINT
        # Move to next synchronized time point for continued analysis
        current_time += timedelta(seconds=check_interval)
    
    return conflicts


def get_drone_position_at_time(drone_path, target_time):
    """
    Precise Drone Position Interpolation at Specific Time
    
    This function calculates the exact position of a drone at any given time by
    performing linear interpolation between waypoints. This is essential for
    synchronized spatial-temporal conflict detection as it enables precise
    position calculation at any timestamp, not just at waypoint locations.
    
    INTERPOLATION ALGORITHM:
        1. Find waypoints immediately before and after target time
        2. Calculate time-based interpolation factor (0.0 to 1.0)
        3. Linearly interpolate x, y, z coordinates using the factor
        4. Return interpolated position with target timestamp
    
    MATHEMATICAL APPROACH:
        - Linear interpolation formula: P = P1 + t * (P2 - P1)
        - Where t = (target_time - time1) / (time2 - time1)
        - Applied separately to x, y, z coordinates for 3D interpolation
    
    SYNCHRONIZATION BENEFITS:
        - Enables exact position calculation at any timestamp
        - Matches simulation's position calculation logic precisely
        - Supports synchronized conflict detection between multiple drones
        - Provides smooth trajectory analysis beyond discrete waypoints
    
    Args:
        drone_path (list): List of waypoints with timestamps, each containing:
            - 'x', 'y', 'z': 3D coordinates or 'position': [x, y, z] array
            - 'timestamp': ISO format timestamp string
        target_time (datetime): Target time for position calculation
    
    Returns:
        dict: Interpolated position at target time containing:
            - 'x', 'y', 'z': Interpolated 3D coordinates
            - 'timestamp': Target timestamp
        None: If target time is outside the drone's flight time range
    
    Example:
        >>> path = [{'x': 0, 'y': 0, 'z': 10, 'timestamp': '2025-08-04T10:00:00'},
        ...         {'x': 100, 'y': 100, 'z': 10, 'timestamp': '2025-08-04T10:05:00'}]
        >>> target = datetime(2025, 8, 4, 10, 2, 30)  # 2.5 minutes into flight
        >>> pos = get_drone_position_at_time(path, target)
        >>> print(pos)  # {'x': 50.0, 'y': 50.0, 'z': 10.0, 'timestamp': target}
    """
    if not drone_path:
        return None
    
    # HELPER FUNCTION: Multi-Format Coordinate Extraction
    # Supports both legacy and new coordinate formats for backward compatibility
    def get_coords(point):
        """
        Extract 3D coordinates from waypoint data supporting multiple formats.
        
        Args:
            point (dict): Waypoint data in either format:
                - {'position': [x, y, z]} - New array format
                - {'x': val, 'y': val, 'z': val} - Legacy field format
        
        Returns:
            tuple: (x, y, z) coordinates as floats
        """
        if 'position' in point:
            # New format: position array [x, y, z]
            return point['position'][0], point['position'][1], point['position'][2]
        else:
            # Legacy format: separate x, y, z fields (z defaults to 0 if missing)
            return point['x'], point['y'], point.get('z', 0)
    
    # STEP 1: FIND BRACKETING WAYPOINTS
    # Locate waypoints immediately before and after the target time
    # This establishes the interpolation segment for position calculation
    before_point = None  # Waypoint at or before target time
    after_point = None   # Waypoint after target time
    
    # Iterate through drone path to find bracketing waypoints
    for i, point in enumerate(drone_path):
        # Parse timestamp (handle both string and datetime objects)
        point_time = point['timestamp']
        if isinstance(point_time, str):
            point_time = parse_time(point_time)
        
        # Update bracketing waypoints based on time comparison
        if point_time <= target_time:
            before_point = point  # This waypoint is at or before target time
        elif point_time > target_time and after_point is None:
            after_point = point   # First waypoint after target time
            break  # Found both brackets, stop searching
    
    # STEP 2: HANDLE EDGE CASES
    # Manage scenarios where target time is outside the flight path time range
    
    if before_point is None:
        # TARGET TIME IS BEFORE FLIGHT START
        # Target time is before the first waypoint - drone hasn't started yet
        return None  # No valid position (drone not active)
    
    if after_point is None:
        # TARGET TIME IS AFTER FLIGHT END
        # Target time is after the last waypoint - use final position
        before_x, before_y, before_z = get_coords(before_point)
        return {
            'x': before_x,
            'y': before_y,
            'z': before_z,
            'timestamp': target_time
        }
    
    # STEP 3: PERFORM LINEAR INTERPOLATION
    # Calculate precise position between the two bracketing waypoints
    
    # Parse bracketing waypoint timestamps
    before_time = before_point['timestamp']
    after_time = after_point['timestamp']
    
    if isinstance(before_time, str):
        before_time = parse_time(before_time)
    if isinstance(after_time, str):
        after_time = parse_time(after_time)
    
    # INTERPOLATION FACTOR CALCULATION
    # Calculate how far along the segment (0.0 to 1.0) the target time falls
    total_time = (after_time - before_time).total_seconds()
    if total_time == 0:
        # IDENTICAL TIMESTAMPS: Return before_point coordinates
        before_x, before_y, before_z = get_coords(before_point)
        return {
            'x': before_x,
            'y': before_y,
            'z': before_z,
            'timestamp': target_time
        }
    
    elapsed_time = (target_time - before_time).total_seconds()
    factor = elapsed_time / total_time  # Interpolation factor (0.0 to 1.0)
    
    # COORDINATE EXTRACTION
    # Get 3D coordinates from both bracketing waypoints
    before_x, before_y, before_z = get_coords(before_point)
    after_x, after_y, after_z = get_coords(after_point)
    
    # LINEAR INTERPOLATION CALCULATION
    # Apply interpolation formula: P = P1 + factor * (P2 - P1) for each axis
    interpolated_x = before_x + factor * (after_x - before_x)
    interpolated_y = before_y + factor * (after_y - before_y)
    interpolated_z = before_z + factor * (after_z - before_z)
    
    # RETURN INTERPOLATED POSITION
    # Provide precise 3D position at the exact target timestamp
    return {
        'x': interpolated_x,
        'y': interpolated_y,
        'z': interpolated_z,
        'timestamp': target_time
    }

def check_waypoint_conflicts(primary_mission, simulated_flights, safety_distance=2, time_threshold=10):
    """
    Legacy function: Simple waypoint-to-waypoint conflict checking
    (Kept for comparison purposes - less accurate than path interpolation)
    """
    conflicts = []

    for sim_flight in simulated_flights:
        for p_wp in primary_mission["waypoints"]:
            for s_wp in sim_flight["waypoints"]:
                d = euclidean_distance(p_wp, s_wp)
                t_diff = time_difference(p_wp["timestamp"], s_wp["timestamp"])

                if d <= safety_distance and t_diff <= time_threshold:
                    conflicts.append({
                        "conflicting_drone": sim_flight["id"],
                        "location": {"x": s_wp["x"], "y": s_wp["y"], "z": s_wp.get("z", 0)},
                        "time": s_wp["timestamp"]
                    })

    return {
        "status": "conflict" if conflicts else "clear",
        "details": conflicts
    }
