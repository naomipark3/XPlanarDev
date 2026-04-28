"""
XPlanar Mover Control via ADS
Sends arbitrary (X, Y) position commands to individual movers through GVL_Cmd.
Requires the ST_MoverCommand struct and GVL_Cmd to be added to the TwinCAT project.
"""

import pyads
import time
import math
import heapq
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

    # TwinCAT uses axis-aligned bounding box (AABB) collision checking.
    # Two movers collide if their boxes overlap on BOTH axes simultaneously.
    # AABB_SIZE = mover edge + TwinCAT's internal safety gap.
    # We don't know the exact gap, so we add a conservative buffer.
    TWINCAT_GAP = 10.0                              # estimated internal safety gap (mm)
    AABB_SIZE = MOVER_SIZE + TWINCAT_GAP            # 123 mm — collision if |dx| < this AND |dy| < this

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

    #Path planning (A* grid search)

    GRID_STEP = 10.0  # mm resolution for A* grid (fine enough for 127mm usable width)

    def _clamp_to_workspace(self, x: float, y: float) -> tuple[float, float]:
        """Clamp a point to the safe travel region inside the glass walls.
        (i.e. define max travel bounds)"""
        return (
            max(self.X_MIN, min(self.X_MAX, x)),
            max(self.Y_MIN, min(self.Y_MAX, y)),
        )

    def _snap_to_grid(self, x: float, y: float) -> tuple[float, float]:
        """Snap a coordinate to the nearest grid point anchored at workspace boundaries."""
        #Grid is anchored at (X_MIN, Y_MIN) so boundaries are always valid grid positions
        gx = self.X_MIN + round((x - self.X_MIN) / self.GRID_STEP) * self.GRID_STEP
        gy = self.Y_MIN + round((y - self.Y_MIN) / self.GRID_STEP) * self.GRID_STEP
        return self._clamp_to_workspace(gx, gy)

    def _grid_neighbors(self, node: tuple[float, float]) -> list[tuple[float, float, float]]:
        """Return walkable 4-connected neighbors (axis-aligned only) with step cost.
        Diagonal moves are excluded to avoid TwinCAT's bounding-circle collision
        checks on diagonal trajectories, which require wider clearance than the
        workspace allows. Grid is anchored at workspace boundaries."""
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx = x + dx * self.GRID_STEP
            ny = y + dy * self.GRID_STEP
            #Clamp to boundary (so we can reach X_MIN/X_MAX/Y_MIN/Y_MAX exactly)
            nx = max(self.X_MIN, min(self.X_MAX, nx))
            ny = max(self.Y_MIN, min(self.Y_MAX, ny))
            if (nx, ny) != (x, y):  # avoid self-loops at boundaries
                step = math.hypot(nx - x, ny - y)
                neighbors.append((nx, ny, step))
        return neighbors

    def _is_clear_of_obstacles(self, x: float, y: float,
                                obstacles: list[tuple[float, float]]) -> bool:
        """AABB point check: collision if BOTH |dx| < AABB_SIZE and |dy| < AABB_SIZE. TwinCAT already does this for us internally,
        but this is useful for finding the optimal path with A*."""
        for ox, oy in obstacles:
            if abs(x - ox) < self.AABB_SIZE and abs(y - oy) < self.AABB_SIZE: #does a grid node lie inside any obstacle's foreign AABB region?
                return False #if so, reject node too close to another mover
        return True

    def _segment_clears_obstacles(self, a: tuple[float, float], b: tuple[float, float],
                                   obstacles: list[tuple[float, float]]) -> bool:
        """
        Checks whether an entire motion leg overlaps with an obstacle's swept AABB region;
        used for both direct-path checking and safe waypoint execution.
        """
        for ox, oy in obstacles:
            #Swept x range of mover centers
            x_min = min(a[0], b[0])
            x_max = max(a[0], b[0])
            #Swept y range of mover centers
            y_min = min(a[1], b[1])
            y_max = max(a[1], b[1])

            #AABB overlap check: does obstacle center fall within AABB_SIZE
            #of the swept range on BOTH axes?
            x_overlap = (ox + self.AABB_SIZE > x_min) and (ox - self.AABB_SIZE < x_max)
            y_overlap = (oy + self.AABB_SIZE > y_min) and (oy - self.AABB_SIZE < y_max)

            if x_overlap and y_overlap:
                return False
        return True

    def _astar(self, start: tuple[float, float], goal: tuple[float, float],
               obstacles: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """
        A* search on a grid over the workspace. Returns list of grid waypoints
        from start to goal (inclusive), or empty list if no path found.
        """
        start_g = self._snap_to_grid(*start) #snap start/goal to planning grid
        goal_g = self._snap_to_grid(*goal)

        #If start or goal is inside an obstacle's clearance zone, we can't plan
        #(but we still try — the mover is already there)

        #initialize search here
        open_set = [(0.0, start_g)] #nodes to explore
        came_from = {} #how we got there (for path reconstruction)
        g_score = {start_g: 0.0} #cost from start --> node

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set) #expand the node with the lowest total estimated cost
            if current in visited:
                continue
            visited.add(current)

            #Close enough to goal?
            if math.hypot(current[0] - goal_g[0], current[1] - goal_g[1]) < self.GRID_STEP * 0.5: #(not requiring exact equality here but accepting close enough to grid-aligned control)
                #If so, reconstruct path
                path = [goal_g]
                node = current
                while node in came_from: #walk backward using parent pointers, build full path from start --> goal
                    path.append(node)
                    node = came_from[node]
                path.append(start_g)
                path.reverse()
                return path

            for nx, ny, step_cost in self._grid_neighbors(current): #only looking at up/down/left/right moves (NOT diagonal)
                neighbor = (nx, ny)
                if neighbor in visited:
                    continue
                #Allow goal node even if inside clearance zone (TwinCAT enforces final check)
                if neighbor != goal_g and not self._is_clear_of_obstacles(nx, ny, obstacles):
                    continue
                tentative_g = g_score[current] + step_cost #update cost if this path is better
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal_g) #prioritize nodes using distance traveled so far (g) and heuristic (straight-line distance to goal) (priority = cost + heuristic)
                    heapq.heappush(open_set, (f, neighbor))

        print("  A*: no path found!")
        return []

    def _simplify_path(self, path: list[tuple[float, float]],
                        obstacles: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """
        Reduce a dense grid path to the minimum waypoints needed. A grid path is 
        many small steps (every 10 mm) used for planning only/follows the grid exactly [(x0, y0), 
        (x1, y1), (x2, y2), ...]
        where waypoints only contain the key turning points so that each segment is one
        actualy move command [(xA, yA), (xB, yB), (goal)]
        Only collapses collinear (axis-aligned) segments to avoid creating
        diagonal moves, which require wider clearance in TwinCAT.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            nxt = path[i + 1]
            #Keep this point if direction changes (not collinear)
            same_x = (prev[0] == path[i][0] == nxt[0])
            same_y = (prev[1] == path[i][1] == nxt[1])
            if not (same_x or same_y):
                simplified.append(path[i])
        simplified.append(path[-1])
        return simplified

    def _plan_waypoints(self, start: tuple[float, float], goal: tuple[float, float],
                        obstacles: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """
        Plan waypoints from start to goal avoiding all obstacles. If a direct segment is safe, go straight
        to the goal. Otherwise, run A* on a grid. Simplify the raw grid path into an executable path. Converts
        simplified path into axis-aligned moves, and ensures the final move to the exact goal does not introduce a 
        diagonal.
        then simplifies to minimal straight-line segments.
        Returns list of waypoints (excluding start, including goal).
        """
        #Check if direct path is clear first
        if self._segment_clears_obstacles(start, goal, obstacles):
            return [goal]

        raw_path = self._astar(start, goal, obstacles)
        if not raw_path:
            #Fallback: just try the direct move and let TwinCAT reject if needed
            print("  No path found, attempting direct move")
            return [goal]

        simplified = self._simplify_path(raw_path, obstacles) #simplify the raw grid path (turns grid path found by A* into turning points)

        #drop the start point (we're already there), keep the rest
        waypoints = simplified[1:]

        #Replace last grid-snapped waypoint with exact goal.
        #If the exact goal differs in both x and y from the prior waypoint,
        #split into two axis-aligned legs to avoid a diagonal.
        #This basically converts the simplified path into executable axis-aligned moves
        if waypoints:
            last_grid = waypoints[-1]
            waypoints.pop()
            prev = waypoints[-1] if waypoints else start
            dx_diff = abs(goal[0] - prev[0]) > 1e-3
            dy_diff = abs(goal[1] - prev[1]) > 1e-3
            if dx_diff and dy_diff:
                #Insert intermediate: match goal's x first, then goal's y
                waypoints.append((goal[0], prev[1]))
            waypoints.append(goal)

        print(f"  A* path ({len(raw_path)} grid nodes -> {len(waypoints)} waypoints):")
        for i, wp in enumerate(waypoints):
            print(f"    wp {i+1}: ({wp[0]:.1f}, {wp[1]:.1f})")
        return waypoints #returns in the form: [(x1, y1), (x2, y2), ..., (goal)] where each tuple 

    #Smart move

    def smart_move_to(self, mover_id: int, x: float, y: float,
                      velocity: float = 300.0, accel: float = 2000.0, decel: float = 2000.0,
                      timeout: float = 30.0) -> bool:
        """
        Move a mover to (x, y), automatically routing around other movers.
        Reads all mover positions, checks for collisions on the straight-line path,
        and inserts waypoints if needed. Each leg uses the existing move_to().
        """
        x, y = self._clamp_to_workspace(x, y) #clamp target to workspace 

        #read current mover positions
        positions = self.get_all_mover_positions()
        start = positions[mover_id]
        goal = (x, y)

        print(f"smart_move_to: Mover {mover_id} from ({start[0]:.1f}, {start[1]:.1f}) "
              f"to ({goal[0]:.1f}, {goal[1]:.1f})")

        #Collect all other mover positions as obstacles (build an obstacle list from the other movers)
        obstacles = [pos for mid, pos in positions.items() if mid != mover_id]
        waypoints = self._plan_waypoints(start, goal, obstacles) #call planner to get waypoints

        if len(waypoints) == 1:
            print(f"  Direct path to goal")

        #**Execute each waypoint one leg at a time using move_to
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
        ams_net_id="169.254.137.138.1.1", #unchanged on ubuntu
        port=852,
        local_ip="192.168.1.1" #changed for ubuntu
    )
    system.connect()
    #system.initialize()

    try:
        positions = system.get_all_mover_positions()
        for m, (px, py) in positions.items():
            print(f"Mover {m} at ({px:.1f}, {py:.1f})")

        #Example: move mover 1 to a target, routing around mover 2 if needed
        system.smart_move_to(2, 56.5, 360.0)
        #system.smart_move_to(2, 180.0, 350.0, velocity=10)

    finally:
        system.disconnect()
