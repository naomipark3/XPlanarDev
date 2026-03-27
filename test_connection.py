import pyads

plc = pyads.Connection('169.254.137.138.1.1', 851, '172.24.68.147')
plc.open()

print(plc.read_state())

n = plc.read_by_name("GVL_Movers.nMoverCount", pyads.PLCTYPE_INT)
print(n)

x = plc.read_by_name("GVL_Movers.aMovers[1].fPosX", pyads.PLCTYPE_LREAL)
print(x)

plc.close()