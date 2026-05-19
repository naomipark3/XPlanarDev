import pyads
plc = pyads.Connection("169.254.137.138.1.1", 852, "192.168.1.1")
plc.open()
print("A:", plc.read_by_name("GVL_Movers.aMovers[1].fPosA", pyads.PLCTYPE_LREAL))
print("B:", plc.read_by_name("GVL_Movers.aMovers[1].fPosB", pyads.PLCTYPE_LREAL))
print("C:", plc.read_by_name("GVL_Movers.aMovers[1].fPosC", pyads.PLCTYPE_LREAL))
plc.close()