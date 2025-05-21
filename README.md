
# Drone Capture Mechanism Simulation in ROS 2 & Gazebo


## Project Summary

This project simulates a quadrotor drone equipped with an umbrella-style net mechanism designed to capture other aerial objects in mid-flight. The system is implemented using **ROS 2 Humble**, and **Gazebo Classic**, with flight dynamics integrated via custom thrust plugin and joint torques controlled through joint plugin. Control performance is benchmarked using a Simulink-tuned PID model. The simulation incorporates complete physical modeling: all components of the drone and the capture mechanism are assigned accurate masses and inertias based on real geometry and materials, including the drone body, arms, slider, column, and braces. Gravitational forces, aerodynamic drag, rotor-generated thrust, and torque are simulated using defined coefficients, allowing for realistic behaviour under various operating conditions. 

The primary objective of this simulation is to evaluate the operational feasibility of the drone and its capture mechanism in a physics-based environment, enabling direct comparison with analytical control performance from Simulink. Unlike the idealized PID design space, the Gazebo simulation includes dynamic effects such as shifting mass distributions, controller delays, and interaction forces between moving joints.

The umbrella-style net mechanism is actuated through a prismatic and revolute joint system, allowing for realistic deployment testing during flight. The system can be easily extended to test different payloads by modifying the drone's URDF/Xacro configuration, specifically by adjusting the mass and inertia parameters of the payload or net structure. The simulation supports testing joint limits, observing deployment dynamics, and detecting failures such as instability, oscillations, or excessive mechanical stress. This environment thus bridges theoretical tuning with real-world deployment, offering a highly detailed and extensible platform for drone capture mechanism validation. 

Note: Currently, the simulation setup demonstrates the drone and capture mechanism in a stable configuration, with the net mechanism shown in both deployed (open) and retracted (closed) states through modeled behaviour. 

---

## Project Structure

```
drone_ws/ 
├── src/
│   ├── drone_bringup/                              # ROS 2 package for launching, GUI control, and node management
│   │   ├── drone_bringup/                          # Python module containing all runtime nodes
│   │   │   ├── __init__.py                         # Required to make it a Python package (empty)
│   │   │   ├── dummy_imu_publisher.py              # Simulated IMU data publisher (test-only)
│   │   │   ├── dummy_joint_state_publisher.py      # Simulated joint state publisher (test-only)
│   │   │   ├── init_joint_positions.py             # Sets the mechanism to its neutral starting position
│   │   │   ├── mechanism_gui.py                    # GUI for toggling mechanism deployment
│   │   │   ├── pid_control_node.py                 # Publishes thrust commands to each rotor using PID
│   │   │   ├── slider_control_service.py           # Service node to control slider effort
│   │   │   └── toggle_mechanism_service.py         # Handles GUI-triggered deployment/retraction
│   │   ├── launch/                                 # ROS 2 launch files for Gazebo, RViz, and system setup
│   │   │   ├── control_and_spawn.launch.py         # Main launch file for simulation
│   │   │   ├── gazebo.launch.py                    # Launches Gazebo Classic (currently empty)
│   │   │   └── rviz_view.launch.py                 # Launches RViz2 with predefined config
│   │   ├── rviz/                                   
│   │   │   └── drone_view.rviz                     # RViz2 configuration file with TF and joint states
│   │   ├── test/                                   # Placeholder for future unit tests (currently empty)
│   │   ├── CMakeLists.txt                          # Build instructions for colcon
│   │   ├── package.xml                             # ROS 2 package manifest
│   │   └── setup.py                                # Python install script for nodes
│   │
│   ├── drone_description/                          # ROS 2 package with URDF model and mesh assets
│   │   ├── meshes/                                 # All .dae files for drone and mechanism components
│   │   ├── urdf/                                   
│   │   │   ├── drone_system.urdf.xacro             # All .dae files for drone and mechanism components
│   │   │   └── macros.urdf.xacro                   # Reusable macros for arms, braces, and joints
│   │   ├── CMakeLists.txt                          # Build instructions for colcon
│   │   ├── package.xml                             # ROS 2 package manifest
│   │   └── setup.py                                # Python install script for nodes (unused)
│   │
│   ├── joint_effort_plugin/                        # Custom Gazebo plugin to control prismatic joint via effort
│   │   ├── src/
│   │   │   └── JointEffortPlugin.cpp               # C++ implementation of the effort-based joint control
│   │   ├── CMakeLists.txt                          # Build instructions for plugin
│   │   └── package.xml                             # ROS 2 plugin package manifest
│   │ 
│   └── thrust_plugin/                              # Custom Gazebo plugin to apply rotor thrust/torque
│       ├── src/
│       │   └── ThrustPlugin.cpp                    # C++ implementation of rotor thrust plugin
│       ├── CMakeLists.txt                          # Build instructions for plugin
│       └── package.xml                             # ROS 2 plugin package manifest
│ 
├── .gitignore                                      # Excludes build, install, logs, temp files, and compiled outputs
├── LICENSE                                         # License file
├── README.md                                       # This readme file with project overview and usage
└── requirements.txt                                # Python dependencies for running nodes and analysis 
```

---

## Joint Setup and Kinematic Behaviour

The drone system includes a deployable umbrella-style net mechanism driven by a four-bar linkage, which is fully modeled in URDF using a combination of prismatic and revolute joints. The overall joint setup reflects realistic mechanical articulation and interaction:

Prismatic Joint (`column_to_slider`):
Controls vertical motion of the central slider. Actuated via effort commands using a custom `JointEffortPlugin`, this joint is responsible for driving the entire deployment process.

Revolute Joints (8 total):
Each brace is connected to the slider and each arm via revolute joints. These joints simulate the four-bar linkage, where the slider’s vertical displacement causes coordinated rotation of braces and arms.

Fixed Joints:
Used to attach dummy links or connect non-actuated structural elements such as the column, base, and drone body. Arms are mounted to dummy links to isolate parent-child conflicts in URDF hierarchy.

This configuration enables a realistic simulation of mechanism deployment dynamics. The kinematic chain ensures symmetric arm motion during actuation, matching the expected physical behavior of the capture mechanism. Joint limits, damping, and inertial properties are defined for all movable elements to ensure numerical stability and plausible motion within Gazebo.

The behavior has been validated through plugin-based deployment tests, confirming that the slider’s motion correctly propagates through the linkage, resulting in smooth arm rotation from closed to open states (and vice versa).

---

## Meshes

All `.dae` mesh files are already inside `drone_description/meshes/`:
- `drone.dae`, `base.dae`, `column.dae`, `slider.dae`
- `arm_1.dae` to `arm_4.dae`
- `brace_1.dae` to `brace_4.dae`

All meshes were aligned and scaled appropriately in Blender before export

---

## System Requirements

To build and run this ROS 2 Gazebo-based drone simulation, the following software and system components are required:

### Operating System
- Ubuntu 22.04 LTS (tested inside Parallels Desktop on macOS with Apple M2 chip)

### ROS 2
- ROS 2 Humble Hawksbill (fully sourced and configured; tested with `colcon` build system)

### Gazebo
- Gazebo Classic 11.15.1, built from source (required due to ARM64 compatibility issues with prebuilt packages)
- gazebo_ros_pkgs, also built from source and correctly linked

### Dependencies
- C++ compiler with CMake ≥ 3.10
- Python 3.10+
- Python packages (`requirements.txt`)

### Recommended Tools
- colcon – ROS 2 build system
- RViz2 – for real-time transform and joint state visualization
- rqt_plot / rqt_graph – for topic monitoring and diagnostics

### Notes
- Ensure environment variables like GAZEBO_PLUGIN_PATH and LD_LIBRARY_PATH are correctly exported in ~/.bashrc
- This setup is tested on ARM64, so x86_64 users may adjust plugin paths and Gazebo installation accordingly

---

## Build Instructions

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Create and enter the workspace
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src

# 3. Clone the repository
git clone https://github.com/01711-01711/drone-simulation.git
mv drone-simulation/* .

# 4. Install dependencies
cd ~/drone_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt

# 5. Build the workspace
colcon build --symlink-install

# 6. Source the workspace
source install/setup.bash

# 7. Make the source persistent in ~/.bashrc:
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
```

---

## Launching the Simulation

1. Run Gazebo manually with the factory plugin:
```bash
gazebo --verbose -s libgazebo_ros_factory.so
```
This starts Gazebo in verbose mode and enables dynamic model spawning from ROS 2

2. Spawn the Drone and Start ROS 2 Nodes:
```bash
cd ~/drone_ws
source install/setup.bash
ros2 launch drone_bringup control_and_spawn.launch.py
```
This will: Spawn the drone model into Gazebo, Initialize joint positions, Start the PID controller node, Load the thrust and joint effort plugins, Launch supporting services and TF broadcasting

3. Open Mechanism GUI:
```bash
ros2 run drone_bringup mechanism_gui.py
```
This opens a simple interface to open or close the net mechanism by sending effort commands to the prismatic joint

---

## Notes

- ros2_control was intentionally avoided due to plugin loading and build issues on ARM64, instead, custom plugins (ThrustPlugin and JointEffortPlugin) are used for direct force and effort application
- Simulation parameters (e.g. thrust coefficients, mass properties, damping) are tuned to match realistic behaviour
- Hovering is not yet functional due to unresolved thrust control loop issues (missing feedback or inconsistent publishing on /rotorX/thrust) and mechanism actuation and deployment staging are fully functional

---

## License

Open for educational use. Developed by UCL Engineering students.

