# Advanced Drone Waypoint Planner & Deconfliction System

A comprehensive interactive drone mission planning system with real-time 4D simulation and conflict detection capabilities.

## 🚀 Quick Start Guide

### Step-by-Step Usage:

1. **Run Primary Plotter:**
   - Execute `python primary_drone_plotter.py`. This opens an interactive interface to create or load the primary drone mission.

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
- **Multi-drone Support**: Manage unlimited drones with individual color coding
- **Dual Input Methods**: 
  - Click-to-plot on 2D visualization
  - Manual coordinate entry with tabular interface
- **Time Management**: Individual start/end times for each drone
- **Real-time Simulation**: 4D visualization with moving spheres and time labels
- **Conflict Detection**: Advanced path interpolation detects conflicts anywhere along flight paths
- **Data Persistence**: Save/load multi-drone missions in JSON format

### Advanced Features
- **Dynamic Drone Management**: Add/remove drones during planning
- **Automatic Timestamp Calculation**: Evenly distributed waypoint timing
- **Visual Trajectory Lines**: Color-coded flight paths
- **Interactive Controls**: Matplotlib-based GUI with comprehensive controls
- **Simulation Integration**: Direct integration with real-time 4D simulator

## 📁 Project Structure

```
flyt_deconfliction/
├── realtime_simulator.py       # 4D visualization engine
├── conflict_checker.py         # Conflict detection algorithm
├── utils.py                   # Essential utility functions (3.1KB)
├── primary_drone_mission.json # Current primary drone mission data
├── simulated_drones.json      # Sample simulated drone missions for testing
├── run_primary_plotter.py     # Simple launcher script
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
python run_primary_plotter.py
```

**Core Workflow:**
1. **Plot Primary Drone Path**: Click on the plot to add waypoints
2. **Set Mission Time Window**: Configure start and end times
3. **Load Other Drones**: Automatically loads from `simulated_drones.json`
4. **Check Conflicts**: Analyze conflicts without running simulation
5. **Run 4D Simulation**: Launch real-time collision detection with visualization

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

## 📋 User Guide

### Interactive Plotter Usage

#### Basic Operations
1. **Add Drones**: Click "Add Drone" to create new drones
2. **Select Drone**: Use radio buttons to switch between drones
3. **Add Waypoints**: Left-click on plot to add waypoints sequentially
4. **Remove Waypoints**: Right-click near waypoints to remove them
5. **Set Times**: Use text boxes to set start/end times per drone

#### Time Format
All times use ISO format: `YYYY-MM-DDTHH:MM:SS`
Example: `2025-08-04T10:00:00`

#### Controls Overview
- **Add Drone**: Create new drone with unique color
- **Remove**: Delete currently selected drone
- **Clear Current**: Remove all waypoints from current drone
- **Clear All**: Remove waypoints from all drones
- **Manual Entry**: Open tabular coordinate editor
- **Save Data**: Export mission to JSON file
- **Load Data**: Import mission from JSON file
- **Run Simulation**: Execute real-time 4D simulation

### Manual Entry Window
- **Time Settings**: Modify start/end times for each drone
- **Waypoint Table**: View and edit coordinates with timestamps
- **Add Waypoint**: Insert waypoints with precise coordinates
- **Remove Selected**: Delete selected waypoints
- **Update Plot**: Sync changes back to main plot

### Mission File Format

```json
{
  "mission_metadata": {
    "created_at": "2025-08-04T12:00:00",
    "drone_count": 2,
    "description": "Mission description"
  },
  "drones": {
    "Drone_Alpha": {
      "waypoints": [
        {"x": 10, "y": 20, "z": 5, "timestamp": "2025-08-04T10:00:00"}
      ],
      "time_window": {
        "start": "2025-08-04T10:00:00",
        "end": "2025-08-04T10:15:00"
      },
      "color": "blue"
    }
  }
}
```

## 🎬 Real-time 4D Simulation

The system features an advanced 4D visualization engine that shows:

### Visual Elements
- **Colored Spheres**: Each drone represented by a sphere in its assigned color
- **Dynamic Time Labels**: Moving timestamps showing current simulation time
- **3D Space**: Full X, Y, Z coordinate visualization
- **Smooth Animation**: Real-time interpolated movement

### Simulation Features
- **Conflict Highlighting**: Red X markers at conflict locations
- **Time Synchronization**: All drones move in synchronized time
- **Interactive Controls**: Pause, speed control, and navigation
- **Legend Display**: Color-coded drone identification

### Technical Details
- Interpolates between waypoints for smooth movement
- Calculates exact positions at any point in time
- Detects conflicts anywhere along flight paths (not just waypoints)
- Supports variable Z-altitudes and different flight speeds

## 🔍 Conflict Detection Algorithm

### Advanced Features
- **Path Interpolation**: Detects conflicts anywhere along flight paths
- **3D Spatial Analysis**: Considers X, Y, and Z coordinates
- **Time Synchronization**: Accurate temporal conflict detection
- **Configurable Safety Distance**: Adjustable minimum separation

### Algorithm Details
1. **Waypoint Interpolation**: Creates dense point clouds along flight paths
2. **Temporal Alignment**: Synchronizes drone positions at each time step
3. **Distance Calculation**: Computes 3D Euclidean distances
4. **Safety Verification**: Checks against minimum separation requirements
5. **Conflict Reporting**: Provides detailed conflict location and timing

## 📊 Example Use Cases

### 1. Multi-Drone Inspection Mission
```bash
# Load pre-configured 3-drone inspection
python simulation_runner.py saved_missions/example_3_drone_mission.json
```

### 2. Conflict Detection Demo
```bash
# Load mission with intentional conflicts
python simulation_runner.py saved_missions/conflict_demo_mission.json
```

### 3. Custom Mission Planning
```bash
# Create custom mission interactively
python interactive_plotter.py
# 1. Add multiple drones
# 2. Click to set waypoints
# 3. Adjust timing
# 4. Save mission
# 5. Run simulation
```

## 🛠️ Advanced Configuration

### Simulation Parameters
Edit `realtime_simulator.py`:
- `SAFETY_DISTANCE`: Minimum separation (default: 2.0 meters)
- `INTERPOLATION_POINTS`: Path density (default: 100 points)
- `SIMULATION_SPEED`: Animation speed multiplier

### Visual Customization
Edit `interactive_plotter.py`:
- `colors`: Drone color palette
- Figure size and layout parameters
- Control button positions and sizes

---

**FlytBase Robotics Assignment 2025** - Advanced Drone Mission Planning & Deconfliction System
