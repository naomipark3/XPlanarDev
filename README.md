The code in this repository follows the architecture shown in the diagram below, where a Python-based ROS-ADS bridge node sends motion commands to the Beckhoff IPC over ADS and receives feedback, enabling closed-loop control of the XPlanar hardware via EtherCAT. 

![Architecture Diagram](Architecture_diagram.drawio.png)

1. Python Interface
Key Python Components

mover_control.py: Core control logic (XPlanarController), including ADS communication, move_to, and smart_move_to (*We focus specifically on the code in mover_control.py for this project).
control_ui.py: User-facing interface for sending commands and interacting with the system
read_mover_data.py: Utility for reading and monitoring real-time mover state from the PLC
setup_ads_route.py: Script to configure the ADS route between the PC and Beckhoff IPC for a Linux-based machine

This project establishes closed-loop control of Beckhoff XPlanar movers using Python by interfacing directly with the Beckhoff IPC over the ADS protocol. The ADS connection is facilitated using pyads. Our Python script, specifically mover_control.py, writes to a custom command structure GVL_Cmd.aMoverCmd defined on the Beckhoff IPC to specify target positions, velocity, and other motion paramters, while real-time mover state (position, busy/done/error flags) is continuously read back from the PLC. The core move_to function sends a position command and then polls the PLC in a loop until the move completes or an error occurs. This enables closed-loop control at the application level. 

The smart_move_to function in mover_control.py extends basic position control by adding navigation between movers. It first reads the current positions of all movers and treats the others as dynamic obstacles using a conservative AABB-based clearance model that matches TwinCAT's internal collision checks. If a direct path to the goal is not safe, it runs an A* search on a discretized workspace grid to generate a collision-free path, then simplifies this into a minimal set of axis-aligned waypoints (i.e. staircasing). Each segment is executed sequentially using the existing move_to closed-loop routine. This ensures safe, stepwise motion while continuously relying on PLC feedback for execution validation.  

2. ROS2 Interface
Key ROS components (in XPlanar_Development\ros2_ws\src\):
- xplanar_bridge.py: Main ROS node (publishes state, exposes service + action server)
- mover_control.py: Python controller handling ADS communication and motion logic
- MoveTo.action: Defines the action interface for motion goals, feedback, and results
- package.xml / setup.py: ROS package configuration and dependencies

The ROS 2 interface is implemented by the XPlanarBridge node (in xplanar_bridge/bridge_node.py), which wraps the XPlanarController and exposes it to the ROS ecosystem. A timer-driven publisher runs at 10 Hz to stream each mover's pose (PoseStamped) on topics like mover_i/pose, providing continuous state feedback. A Trigger service (/initialize) allows external nodes to initialize the system (lift the movers off the track so they are in a state to accept commands), while an action server (move_to) handles motion requests and enables asynchronous goal execution with real-time feedback (i.e. current position and remaning distance) and a final success or failure result. The action server calls smart_move_to, which bridges high-level ROS commands to low-level ADS-based control on the PLC. 
