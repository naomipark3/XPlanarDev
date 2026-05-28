## Overview
The code in this repository follows the architecture shown in the diagram below, where a Python-based ROS-ADS bridge node sends motion commands to the Beckhoff IPC over ADS and receives feedback, enabling closed-loop control of the XPlanar hardware via EtherCAT. 

![Architecture Diagram](Architecture_diagram.drawio.png)

## Python Interface

### Key Components

- xplanar_bridge/xplanar_bridge/mover_control.py: Core control logic (XPlanarController), including ADS communication, move_to, smart_move_to, tilt_to, rotate_to, and move_z (Note: we focus specifically on the code in mover_control.py for this project).
- control_ui.py: User-facing interface for sending commands and interacting with the system
- read_mover_data.py: Utility for reading and monitoring real-time mover state from the PLC
- setup_ads_route.py: Script to configure the ADS route between the PC and Beckhoff IPC for a Linux-based machine

This project establishes closed-loop control of Beckhoff XPlanar movers using Python by interfacing directly with the Beckhoff IPC over the ADS protocol. The ADS connection is facilitated using pyads. Our Python script, specifically mover_control.py, writes to a custom command structure GVL_Cmd.aMoverCmd defined on the Beckhoff IPC to specify target positions, velocity, and other motion parameters, while real-time mover state (position, busy/done/error flags) is continuously read back from the PLC. The core move_to function sends a position command and then polls the PLC in a loop until the move completes or an error occurs. This enables closed-loop control at the application level.
The smart_move_to function in mover_control.py extends basic position control by adding navigation between movers. It first reads the current positions of all movers and treats the others as dynamic obstacles using a conservative AABB-based clearance model that matches TwinCAT's internal collision checks. If a direct path to the goal is not safe, it runs an A* search on a discretized workspace grid to generate a collision-free path, then simplifies this into a minimal set of axis-aligned waypoints (i.e. staircasing). Each segment is executed sequentially using the existing move_to closed-loop routine. This ensures safe, stepwise motion while continuously relying on PLC feedback for execution validation.
In addition to planar motion, mover_control.py also exposes tilt_to and rotate_to functions for commanding mover orientation. The tilt_to function tilts a mover about either its A or B axis (single-axis tilt) by writing target angle and dynamics values to dedicated tilt fields in GVL_Cmd.aMoverCmd and polling the PLC for completion in the same closed-loop pattern as move_to. The rotate_to function rotates a mover about its C axis (yaw) to a target angle, with an optional additional_turns parameter for commanding extra full rotations before settling at the final angle. Both functions rely on the same PLC-side cyclic logic that mirrors the move command pattern: write target + dynamics, set bExecute, monitor bBusy/bDone/bError, then auto-reset on completion.
The move_z function controls the mover's levitation height by commanding vertical motion to a target Z position. Z motion uses an independent execute flag (bExecuteZ) on the PLC side so it can run concurrently with planar moves, but shares the Movement dynamics (velocity, accel, decel) with XY since the PLC does not expose a separate dynamics struct for vertical motion. The Z range is limited to a few millimeters above the nominal levitation gap, consistent with the XPlanar's design.

## ROS 2 Interface

### Key ROS Components (in XPlanar_Development\ros2_ws\src\):

- xplanar_bridge.py: Main ROS node (publishes state, exposes service + action servers)
- mover_control.py: Python controller handling ADS communication and motion logic
- MoveTo.action: Defines the action interface for motion goals, feedback, and results
- TiltTo.action: Defines the action interface for tilt goals, feedback, and results
- RotateTo.action: Defines the action interface for rotation goals, feedback, and results
- MoveZ.action: Defines the action interface for vertical motion goals, feedback, and results
- package.xml / setup.py: ROS package configuration and dependencies

The ROS 2 interface is implemented by the XPlanarBridge node (in xplanar_bridge/bridge_node.py), which wraps the XPlanarController and exposes it to the ROS ecosystem. A timer-driven publisher runs at 10 Hz to stream each mover's full 6-DoF pose (PoseStamped) on topics like mover_i/pose, providing continuous state feedback. The pose's position field reports x, y, z in meters, while the orientation field reports the mover's A/B/C angles converted to a quaternion. A Trigger service (/initialize) allows external nodes to initialize the system (lift the movers off the track so they are in a state to accept commands), while four action servers handle motion requests: move_to for planar navigation, tilt_to for single-axis tilt about A or B, rotate_to for rotation about C, and move_z for vertical motion. Each action server enables asynchronous goal execution with real-time feedback and a final success or failure result. The move_to action calls smart_move_to, while tilt_to, rotate_to, and move_z call their respective controller methods, bridging high-level ROS commands to low-level ADS-based control on the PLC.

## Running the ROS 2 Interface

Navigate to the ROS 2 workspace:

```bash
cd ~/Desktop/XPlanarDev/ros2_ws
```

Compile the ROS package:

```bash
colcon build --packages-select xplanar_interfaces xplanar_bridge --symlink-install
```

Add the compiled ROS package to the current shell environment:

```bash
source install/setup.bash
```

Launch the bridge node:

```bash
ros2 run xplanar_bridge bridge
```

See the current mover state (6-DoF):

```bash
ros2 topic echo /mover_1/pose
```

Initialize the movers/ADS connection (lift movers off the track):

```bash
ros2 service call /initialize std_srvs/srv/Trigger
```

Send a closed-loop navigation command:

```bash
ros2 action send_goal --feedback /move_to xplanar_interfaces/action/MoveTo "{mover_id: 1, x: 56.5, y: 120.0, velocity: 10.0}"
```

Send a closed-loop spin command:
```bash
ros2 action send_goal /rotate_to xplanar_interfaces/action/RotateTo   "{mover_id: 2, angle: 10.15, additional_turns: 0, velocity: 15.0}" --feedback
```

Send a closed-loop tilt command:
```bash
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo   "{mover_id: 1, angle: 0.7, axis: 'A', velocity: 0.5}" --feedback
```

Send a closed-loop vertical move command:
```bash
ros2 action send_goal /move_z xplanar_interfaces/action/MoveZ "{mover_id: 1, z: 6.0, velocity: 50.0}" --feedback
```

The action servers continuously stream feedback during execution. The move_to server reports the mover's current position and remaining distance to the goal, while tilt_to and rotate_to report the mover's current angle on the relevant axis, and move_z reports the mover's current Z position.