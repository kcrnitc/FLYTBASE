# Advanced Drone Waypoint Planner & Deconfliction System

A comprehensive interactive drone mission planning system with real-time 4D simulation and conflict detection capabilities.

## 🚀 Quick Start Guide

### Step-by-Step Usage:

1. **Run Primary Plotter:**
   - Execute `primary_drone_plotter.py`. This opens an interactive interface to create or load the primary drone mission.

2. **Plot the Primary Drone Path:**
   - Click anywhere on the canvas to place a waypoint.

3. **Load Simulated Drones (Optional):**
   - You can load a JSON file containing simulated drone missions to compare against.

4. **Save Mission:**
   - Once satisfied with the path, click the "Save Primary Path" button to export the primary mission as a JSON file.

5. **Check for Conflicts:**
   - Click the "Check Conflicts" button. The system will evaluate potential conflicts between the primary and simulated drones.

6. **Run Simulation:**
   - After conflict detection, click the "Run Simulation" button. A 3D visualization will open showing drone paths over time.
   - Conflicts will be marked with cross symbols for easy identification.

## 🚁 Features

### Core Capabilities
- **Multi-drone**: Manage unlimited drones with individual color coding
- **Dual Input Methods**: 
  - Click-to-plot on 2D visualization
  - Manual coordinate entry with tabular interface
- **Time Management**: Individual start/end times for each drone
- **Real-time Simulation**: 4D visualization with moving spheres and time labels
- **Conflict Detection**: Advanced path interpolation detects conflicts anywhere along flight paths
- **Data Persistence**: Save/load multi-drone missions in JSON format


## 📁 Project Structure

```
flyt_deconfliction/
├── realtime_simulator.py      # 4D visualization 
├── conflict_checker.py        # Conflict detection algorithm
├── utils.py                   # Essential utility functions (3.1KB)
├── primary_drone_mission.json # Current primary drone mission data
├── simulated_drones.json      # Sample simulated drone missions for testing
└── README.md                  # This documentation file
```

## 🚀 Getting Started

### Prerequisites
```bash
pip install matplotlib numpy tkinter
```

### Quick Start - Primary Drone Plotter

**Launch the Primary Drone Plotter Interface:**
```bash
python primary_drone_plotter.py
```

**Core Workflow:**
1. **Plot Primary Drone Path**: Click on the plot to add waypoints
2. **Set Mission Time Window**: Configure start and end times
3. **Load Other Drones**: Automatically loads from `simulated_drones.json`
4. **Check Conflicts**: Analyze conflicts without running simulation
5. **Run 4D Simulation**: Launch real-time collision detection with visualization
6. **Update Waypoints**: Change the Json file to update waypoints.

### Advanced Usage

**Direct API Usage:**
```python
from conflict_checker import check_deconfliction

# Load mission data
primary_mission = {'waypoints': [...]}
simulated_flights = [{'drone_id': 'SIM_001', 'waypoints': [...]}]

# Check for conflicts
result = check_deconfliction(primary_mission, simulated_flights, safety_distance=2.0)
print(f"Status: {result['status']}")  # "clear" or "conflict"
for conflict in result['details']:
    print(f"Conflict with {conflict['conflicting_drone']} at {conflict['time']}")
```



#### Time Format
All times use ISO format: `YYYY-MM-DDTHH:MM:SS`
Example: `2025-08-04T10:00:00`



## 🛠️ Advanced Configuration

### Simulation Parameters
Edit `realtime_simulator.py`:
- `SAFETY_DISTANCE`: Minimum separation (default: 2.0 meters)
- `INTERPOLATION_POINTS`: Path density (default: 100 points)
- `SIMULATION_SPEED`: Animation speed multiplier


---

**FlytBase Robotics Assignment 2025** - Advanced Drone Mission Planning & Deconfliction System 
