#!/usr/bin/env python3
# run_primary_plotter.py - Launcher for Primary Drone Plotter

"""
Simple launcher script for the Primary Drone Plotter interface.
This provides a clean entry point for the simplified interface.
"""

import sys
import os

# Add current directory to path to ensure imports work
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from primary_drone_plotter import main

if __name__ == "__main__":
    print("Starting Primary Drone Plotter...")
    print("=" * 50)
    print("FEATURES:")
    print("• Plot primary drone path by clicking")
    print("• Save path as primary_drone_mission.json")
    print("• Load other drones from JSON files")
    print("• Check conflicts without running simulation")
    print("=" * 50)
    main()
