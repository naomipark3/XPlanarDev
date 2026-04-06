"""
XPlanar Mover Control via ADS
Sends arbitrary (X, Y) position commands to individual movers through GVL_Cmd.
Requires the ST_MoverCommand struct and GVL_Cmd to be added to the TwinCAT project.
"""

import pyads
import time
import math
from dataclasses import dataclass


@dataclass
class MoverStatus:
    busy: bool
    done: bool
    error: bool
    x: float
    y: float
    z: float

XPLANAR_ERRORS = {
    33105: "33105: Command not allowed in current mode. System must be initalized and movers must be lifted off the track.",
    33155: "33155: Target position out of bounds",
    33158: "33158: Move would collide with another mover",
}

class XPlanarController:

    # Workspace geometry (mm)
    WORKSPACE_X = 240.0     # total x extent of tile layout
    WORKSPACE_Y = 480.0     # total y extent of tile layout
    MOVER_SIZE  = 113.0     # mover footprint edge length
    NUM_MOVERS  = 2

    # TwinCAT treats movers as bounding circles (diameter = diagonal of square).
    # Diagonal of 113 mm square = 113 * sqrt(2) ≈ 159.8 mm.
    # Min center-to-center = one full diagonal + safety buffer.
    MOVER_DIAGONAL = MOVER_SIZE * 1.4143            # ~159.8 mm
    SAFETY_MARGIN = 20.0                            # mm buffer beyond contact
    MIN_CLEARANCE = MOVER_DIAGONAL + SAFETY_MARGIN  # ~180 mm center-to-center

    # Mover centers can't get closer than half a mover width to the glass walls.
    WALL_MARGIN = MOVER_SIZE / 2.0                  # 56.5 mm
    X_MIN = WALL_MARGIN
    X_MAX = WORKSPACE_X - WALL_MARGIN
    Y_MIN = WALL_MARGIN
    Y_MAX = WORKSPACE_Y - WALL_MARGIN

    def __init__(self, ams_net_id: str, port: int = 852, local_ip: str = None):
        self.ams_net_id = ams_net_id
        self.port = port
        self.local_ip = local_ip
        self.plc: pyads.Connection = None

    def connect(self):
        '''Establish PyADS connection using the ADS route we established.'''
        self.plc = pyads.Connection(self.ams_net_id, self.port, self.local_ip)
        self.plc.open()
        print(f"Connected to {self.ams_net_id}:{self.port}")

    def initialize(self):
        """Toggle ResetPressed to prepare system for Start."""
        symbol = "MAIN.ControlSourceHMI.MainPMLControl_Simplified.ResetPressed"

        print("ResetPressed --> TRUE")
        self.plc.write_by_name(symbol, True, pyads.PLCTYPE_BOOL)
        time.sleep(3.0)

        print("ResetPressed --> FALSE")
        self.plc.write_by_name(symbol, False, pyads.PLCTYPE_BOOL)
        time.sleep(3.0)

        print("ResetPressed --> TRUE")
        self.plc.write_by_name(symbol, True, pyads.PLCTYPE_BOOL)
        time.sleep(7.0)

        print("System initialized, movers lifted from track")

    def disconnect(self):
        if self.plc:
            self.plc.close()
            self.plc = None
            print("Disconnected")

    def get_mover_position(self, mover_id: int) -> tuple[float, float, float]:
        """Read current (x, y, z) position of a mover."""
        prefix = f"GVL_Movers.aMovers[{mover_id}]"
        x = self.plc.read_by_name(f"{prefix}.fPosX", pyads.PLCTYPE_LREAL)
        y = self.plc.read_by_name(f"{prefix}.fPosY", pyads.PLCTYPE_LREAL)
        z = self.plc.read_by_name(f"{prefix}.fPosZ", pyads.PLCTYPE_LREAL)
        return x, y, z

    def get_all_mover_positions(self) -> dict[int, tuple[float, float]]:
        """Read (x, y) positions of all movers. Returns {mover_id: (x, y)}."""
        positions = {}
        for m in range(1, self.NUM_MOVERS + 1):
            x, y, _ = self.get_mover_position(m)
            positions[m] = (x, y)
        return positions

    def get_cmd_status(self, mover_id: int) -> MoverStatus:
        """Read the command interface status for a mover."""
        prefix = f"GVL_Cmd.aMoverCmd[{mover_id}]"
        x, y, z = self.get_mover_position(mover_id)
        return MoverStatus(
            busy=self.plc.read_by_name(f"{prefix}.bBusy", pyads.PLCTYPE_BOOL),
            done=self.plc.read_by_name(f"{prefix}.bDone", pyads.PLCTYPE_BOOL),
            error=self.plc.read_by_name(f"{prefix}.bError", pyads.PLCTYPE_BOOL),
            x=x, y=y, z=z,
        )

    # ── Path planning helpers ────────────────────────────────────────────

    def _clamp_to_workspace(self, x: float, y: float) -> tuple[float, float]:
        """Clamp a point to the safe travel region inside the glass walls."""
        return (
            max(self.X_MIN, min(self.X_MAX, x)),
            max(self.Y_MIN, min(self.Y_MAX, y)),
        )

    @staticmethod
    def _point_to_segment_dist(px: float, py: float,
                                ax: float, ay: float,
                                bx: float, by: float) -> float:
        """Minimum distance from point (px, py) to line segment (a -> b)."""
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-9:
            # Segment is a point
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def _path_collides(self, start: tuple[float, float], goal: tuple[float, float],
                       obstacle: tuple[float, float]) -> bool:
        """Does the straight-line path from start to goal pass within MIN_CLEARANCE of obstacle?"""
        dist = self._point_to_segment_dist(
            obstacle[0], obstacle[1],
            start[0], start[1],
            goal[0], goal[1],
        )
        return dist < self.MIN_CLEARANCE

    def _plan_waypoints(self, start: tuple[float, float], goal: tuple[float, float],
                        obstacle: tuple[float, float]) -> list[tuple[float, float]]:
        """
        If the direct path collides with the obstacle, route around using
        axis-aligned (L-shaped) waypoints so each leg is purely X or Y
        motion — no diagonal sweeps past the obstacle.

        Generates candidate routes and picks the first whose every leg
        clears the obstacle.

        Returns a list of (x, y) waypoints including the final goal.
        """
        if not self._path_collides(start, goal, obstacle):
            return [goal]

        clearance = self.MIN_CLEARANCE + 20.0  # comfortable margin

        # Build candidate L-shaped routes (each is a list of waypoints ending at goal).
        candidates = []

        # Option A: Y first, then X — go to (start.x, goal.y) then goal
        wp_a = self._clamp_to_workspace(start[0], goal[1])
        candidates.append([wp_a, goal])

        # Option B: X first, then Y — go to (goal.x, start.y) then goal
        wp_b = self._clamp_to_workspace(goal[0], start[1])
        candidates.append([wp_b, goal])

        # Option C: wide left — x = obstacle.x - clearance, 3-leg route
        safe_x_left = obstacle[0] - clearance
        candidates.append([
            self._clamp_to_workspace(safe_x_left, start[1]),
            self._clamp_to_workspace(safe_x_left, goal[1]),
            goal,
        ])

        # Option D: wide right — x = obstacle.x + clearance, 3-leg route
        safe_x_right = obstacle[0] + clearance
        candidates.append([
            self._clamp_to_workspace(safe_x_right, start[1]),
            self._clamp_to_workspace(safe_x_right, goal[1]),
            goal,
        ])

        def route_is_clear(waypoints):
            prev = start
            for wp in waypoints:
                if self._path_collides(prev, wp, obstacle):
                    return False
                prev = wp
            return True

        def wall_clearance(waypoints):
            return min(
                min(p[0] - self.X_MIN, self.X_MAX - p[0],
                    p[1] - self.Y_MIN, self.Y_MAX - p[1])
                for p in waypoints
            )

        valid = [(r, wall_clearance(r)) for r in candidates if route_is_clear(r)]

        if valid:
            best = max(valid, key=lambda x: x[1])[0]
        else:
            # Last resort: go to corner farthest from obstacle, then goal
            corners = [
                (self.X_MIN, self.Y_MIN), (self.X_MAX, self.Y_MIN),
                (self.X_MIN, self.Y_MAX), (self.X_MAX, self.Y_MAX),
            ]
            corners.sort(key=lambda c: math.hypot(c[0] - obstacle[0], c[1] - obstacle[1]),
                         reverse=True)
            best = [corners[0], goal]
            print(f"  Warning: corner fallback via ({corners[0][0]:.1f}, {corners[0][1]:.1f})")

        print(f"  Routing around obstacle at ({obstacle[0]:.1f}, {obstacle[1]:.1f}):")
        for i, wp in enumerate(best):
            print(f"    wp {i+1}: ({wp[0]:.1f}, {wp[1]:.1f})")
        return best

    # ── Smart move ───────────────────────────────────────────────────────

    def smart_move_to(self, mover_id: int, x: float, y: float,
                      velocity: float = 300.0, accel: float = 2000.0, decel: float = 2000.0,
                      timeout: float = 30.0) -> bool:
        """
        Move a mover to (x, y), automatically routing around other movers.
        Reads all mover positions, checks for collisions on the straight-line path,
        and inserts waypoints if needed. Each leg uses the existing move_to().
        """
        x, y = self._clamp_to_workspace(x, y)

        # Get current positions of all movers
        positions = self.get_all_mover_positions()
        start = positions[mover_id]
        goal = (x, y)

        print(f"smart_move_to: Mover {mover_id} from ({start[0]:.1f}, {start[1]:.1f}) "
              f"to ({goal[0]:.1f}, {goal[1]:.1f})")

        # Check against every other mover
        waypoints = [goal]
        for other_id, other_pos in positions.items():
            if other_id == mover_id:
                continue
            waypoints = self._plan_waypoints(start, goal, other_pos)
            # For 2 movers this loop runs once; for N movers you'd want
            # a more sophisticated multi-obstacle planner here.
            break

        if len(waypoints) == 1 and waypoints[0] == goal:
            print(f"  Direct path is clear")

        # Execute each leg sequentially
        for i, (wx, wy) in enumerate(waypoints):
            leg_label = f"leg {i+1}/{len(waypoints)}"
            print(f"  Executing {leg_label}: -> ({wx:.1f}, {wy:.1f})")
            ok = self.move_to(mover_id, wx, wy,
                              velocity=velocity, accel=accel, decel=decel,
                              block=True, timeout=timeout)
            if not ok:
                print(f"  {leg_label} FAILED — aborting smart_move_to")
                return False

        return True

    #Existing move_to (unchanged)

    def move_to(self, mover_id: int, x: float, y: float, velocity: float = 300.0, accel: float = 2000.0, decel: float = 2000.0, 
                block: bool = True, timeout: float = 30.0, poll_interval: float = 0.05) -> bool:
        """
        Command a mover to an (x, y) position.
        Args:
        - mover_id: 1-based mover index
        - x, y: target position in mm
        - block: if True, wait until move completes or times out
        - timeout: max seconds to wait (only if block=True)
        - poll_interval: seconds between status polls

        Returns:
        True if move completed successfully (or if non-blocking),
        False if error or timeout.
        """
        prefix = f"GVL_Cmd.aMoverCmd[{mover_id}]"

        #Check not already busy
        status = self.get_cmd_status(mover_id)
        if status.busy:
            print(f"Mover {mover_id} is busy, waiting...")
            if not self._wait_not_busy(mover_id, timeout):
                print(f"Mover {mover_id} still busy after {timeout}s")
                return False

        #Write target and trigger
        self.plc.write_by_name(f"{prefix}.fTargetX", x, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name(f"{prefix}.fTargetY", y, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name(f"{prefix}.fVelocity", velocity, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name(f"{prefix}.fAccel", accel, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name(f"{prefix}.fDecel", decel, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name(f"{prefix}.bExecute", True, pyads.PLCTYPE_BOOL)
        print(f"Mover {mover_id} -> ({x:.1f}, {y:.1f})")

        if not block:
            return True

        #Wait for completion
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            status = self.get_cmd_status(mover_id)
            print(f"  Mover {mover_id}: ({status.x:.1f}, {status.y:.1f}, {status.z:.1f})")
            if status.done:
                self.plc.write_by_name(f"{prefix}.bExecute", False, pyads.PLCTYPE_BOOL)
                print(f"Mover {mover_id} arrived at ({status.x:.1f}, {status.y:.1f})")
                return True
            if status.error:
                error_id = self.plc.read_by_name(f"{prefix}.nErrorID", pyads.PLCTYPE_UDINT)
                msg = XPLANAR_ERRORS.get(error_id, f"Unknown error {error_id}")
                self.plc.write_by_name(f"{prefix}.bExecute", False, pyads.PLCTYPE_BOOL)
                print(f"Mover {mover_id} rejected: {msg}")
                return False
            time.sleep(poll_interval)

        #Timeout
        self.plc.write_by_name(f"{prefix}.bExecute", False, pyads.PLCTYPE_BOOL)
        print(f"Mover {mover_id} TIMEOUT after {timeout}s")
        return False

    def _wait_not_busy(self, mover_id: int, timeout: float) -> bool:
        '''Polls bBusy every 50 ms until the mover is no longer busy (returns True) 
        or timeout expires (returns False).'''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not self.plc.read_by_name(
                f"GVL_Cmd.aMoverCmd[{mover_id}].bBusy", pyads.PLCTYPE_BOOL
            ):
                return True
            time.sleep(0.05)
        return False


if __name__ == "__main__":
    system = XPlanarController(
        ams_net_id="169.254.137.138.1.1",
        port=852,
        local_ip="172.24.68.147",
    )
    system.connect()

    try:
        positions = system.get_all_mover_positions()
        for m, (px, py) in positions.items():
            print(f"Mover {m} at ({px:.1f}, {py:.1f})")

        # Example: move mover 1 to a target, routing around mover 2 if needed
        system.smart_move_to(1, 180.0, 56.5, velocity=20)

    finally:
        system.disconnect()