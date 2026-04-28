import pyads

'''Set up an ADS route from your local machine to the Beckhoff IPC with pyads. 
As a result, we can avoid setting up an ADS route on the Beckhoff IPC using TwinCAT.'''

# 1. Tell pyads what local AMS Net ID to use
pyads.open_port()
pyads.set_local_address("192.168.1.2.1.1")
pyads.close_port()

# 2. Push a route to the IPC so it knows how to reach this Ubuntu machine. THIS WORKED!!
pyads.add_route_to_plc(
    sending_net_id="192.168.1.2.1.1",
    adding_host_name="192.168.1.2",
    ip_address="192.168.1.1",        # IPC's direct-link IP
    username="Administrator",
    password="1",                     # replace with your IPC admin password
    route_name="ubuntu-src",
)

# plc = pyads.Connection("169.254.137.138.1.1", pyads.PORT_SYSTEMSERVICE, "192.168.1.1")
# plc.open()
# print("Device info:", plc.read_device_info())
# plc.close()

#3. Test the connection (need to open port 852 on TwinCAT side)
plc = pyads.Connection("169.254.137.138.1.1", 852, "192.168.1.1") #port 852
plc.open()
print("PLC state:", plc.read_state())
plc.close()