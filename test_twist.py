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
    33105: "Command not allowed in current mode",
    33155: "Target position out of bounds",
    33158: "Move would collide with another mover",
}

class XPlanarController:
    def __init__(self, ams_net_id: str, port: int = 852, local_ip: str = None):
        self.ams_net_id = ams_net_id
        self.port = port
        self.local_ip = local_ip
        self.plc: pyads.Connection = None

    def connect(self):
        self.plc = pyads.Connection(self.ams_net_id, self.port, self.local_ip)
        self.plc.open()
        print(f"Connected to {self.ams_net_id}:{self.port}")

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

    def move_to(self, mover_id: int, x: float, y: float, block: bool = True,
                timeout: float = 10.0, poll_interval: float = 0.05) -> bool:
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
    ctrl = XPlanarController(
        ams_net_id="169.254.137.138.1.1",
        port=852,
        local_ip="172.24.68.147",
    )
    ctrl.connect()

    try:
            for m in (1, 2):
                pos = ctrl.get_mover_position(m)
                print(f"Mover {m} at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

            ctrl.move_to(2, 50.0, 300.0)

    finally:
        ctrl.disconnect()