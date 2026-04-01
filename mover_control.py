"""
XPlanar Mover Control via ADS
Sends arbitrary (X, Y) position commands to individual movers through GVL_Cmd.
Requires the ST_MoverCommand struct and GVL_Cmd to be added to the TwinCAT project.
"""

import pyads
import time
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

    def move_to(self, mover_id: int, x: float, y: float, velocity: float = 300.0, accel: float = 2000.0, decel: float = 2000.0, 
                block: bool = True, timeout: float = 30.0, poll_interval: float = 0.05) -> bool:
        """
        Command a mover to an (x, y) position.

        Args:
            mover_id: 1-based mover index
            x, y: target position in mm
            block: if True, wait until move completes or times out
            timeout: max seconds to wait (only if block=True)
            poll_interval: seconds between status polls

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

        # Timeout
        self.plc.write_by_name(f"{prefix}.bExecute", False, pyads.PLCTYPE_BOOL)
        print(f"Mover {mover_id} TIMEOUT after {timeout}s")
        return False

    def _wait_not_busy(self, mover_id: int, timeout: float) -> bool:
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
    system.initialize() #equivalent to manually toggling ResetPressed. NOTE: this only needs to be pressed once when turned on!
    #sometimes, this takes two tries to work. 

    try:
            for m in (1, 2):
                pos = system.get_mover_position(m)
                print(f"Mover {m} at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

            system.move_to(2, 100.0, 350.0, velocity=10) #mover, x pos, y pos

    finally:
        system.disconnect()