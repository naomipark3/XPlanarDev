#!/bin/bash
# Demo sequence - mirrors mover_control.py __main__ block (only active lines).
# First, make sure to run the bridge node first:
#   cd ~/Desktop/XPlanarDev/ros2_ws
#   ros2 run xplanar_bridge bridge
#Then, in another terminal, we run the following commands (in the ~/Desktop/XPlanarDev/ros2_ws directory):
#   chmod +x demos/demo_routine.sh
#   ./demos/demo_routine.sh

set -e

echo "Initializing system (takes about 13s)"
ros2 service call /initialize std_srvs/srv/Trigger

#smart_move_to(1, 120.0, 120.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 1, x: 120.0, y: 120.0, velocity: 10.0}"

#smart_move_to(2, 120.0, 320.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 2, x: 120.0, y: 320.0, velocity: 10.0}"

#rotate_to(1, 6*math.pi/2)
ros2 action send_goal /rotate_to xplanar_interfaces/action/RotateTo \
  "{mover_id: 1, angle: 9.42477796076938, additional_turns: 0, velocity: 1.0}"

#sleep 0.5
sleep 0.5

#rotate_to(1, 0)
ros2 action send_goal /rotate_to xplanar_interfaces/action/RotateTo \
  "{mover_id: 1, angle: 0.0, additional_turns: 0, velocity: 1.0}"

#sleep 0.5
sleep 0.5

#move_z(1, 6.0)
ros2 action send_goal /move_z xplanar_interfaces/action/MoveZ \
  "{mover_id: 1, z: 6.0, velocity: 50.0}"

#tilt_to(1, 1.0, axis="A")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 1, angle: 1.0, axis: 'A', velocity: 0.5}"

#tilt_to(1, 0.0, axis="A")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 1, angle: 0.0, axis: 'A', velocity: 0.5}"

#sleep 0.5
sleep 0.5

#tilt_to(1, -1.0, axis="A")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 1, angle: -1.0, axis: 'A', velocity: 0.5}"

#tilt_to(1, 0.0, axis="A")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 1, angle: 0.0, axis: 'A', velocity: 0.5}"

#sleep 0.5
sleep 0.5

#move_z(1, 2.0)
ros2 action send_goal /move_z xplanar_interfaces/action/MoveZ \
  "{mover_id: 1, z: 2.0, velocity: 50.0}"

#sleep 1.0
sleep 1.0

#smart_move_to(1, 160.0, 100.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 1, x: 160.0, y: 100.0, velocity: 10.0}"

#sleep 0.5
sleep 0.5

#smart_move_to(2, 80.0, 360.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 2, x: 80.0, y: 360.0, velocity: 10.0}"

#sleep 0.5
sleep 0.5

#rotate_to(2, -6*math.pi/2). This is typically where the script will fail and "setpoint is unreachable"
ros2 action send_goal /rotate_to xplanar_interfaces/action/RotateTo \
  "{mover_id: 2, angle: -9.42477796076938, additional_turns: 0, velocity: 1.0}"

#sleep 0.5
sleep 0.5

#rotate_to(2, 0)
ros2 action send_goal /rotate_to xplanar_interfaces/action/RotateTo \
  "{mover_id: 2, angle: 0.0, additional_turns: 0, velocity: 1.0}"

#sleep 0.5
sleep 0.5

#move_z(2, 6.0)
ros2 action send_goal /move_z xplanar_interfaces/action/MoveZ \
  "{mover_id: 2, z: 6.0, velocity: 50.0}"

#tilt_to(2, 1.0, axis="B")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 2, angle: 1.0, axis: 'B', velocity: 0.5}"

#tilt_to(2, 0.0, axis="B")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 2, angle: 0.0, axis: 'B', velocity: 0.5}"

#sleep 0.5
sleep 0.5

#tilt_to(2, -1.0, axis="B")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 2, angle: -1.0, axis: 'B', velocity: 0.5}"

#tilt_to(2, 0.0, axis="B")
ros2 action send_goal /tilt_to xplanar_interfaces/action/TiltTo \
  "{mover_id: 2, angle: 0.0, axis: 'B', velocity: 0.5}"

#sleep 0.5
sleep 0.5

#move_z(2, 2.0)
ros2 action send_goal /move_z xplanar_interfaces/action/MoveZ \
  "{mover_id: 2, z: 2.0, velocity: 50.0}"

#smart_move_to(1, 120.0, 120.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 1, x: 120.0, y: 120.0, velocity: 10.0}"

#smart_move_to(2, 120.0, 320.0, velocity=10)
ros2 action send_goal /move_to xplanar_interfaces/action/MoveTo \
  "{mover_id: 2, x: 120.0, y: 320.0, velocity: 10.0}"

echo "Demo sequence complete"