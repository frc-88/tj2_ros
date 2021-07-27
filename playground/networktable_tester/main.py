import time
from networktables import NetworkTables

class DummyServer:
    def __init__(self):
        NetworkTables.initialize()
        assert NetworkTables.isServer()

        self.nt = NetworkTables.getTable("coprocessor")

        self.driver_station_table_key = "host/DriverStation"
        self.odom_table_key = "host/odom"

        self.start_time = time.time()
    
    def run(self):
        while True:
            self.nt.getEntry("host/timestamp").setDouble(self.server_time() * 1E6)

            self.publish_fms()

            time.sleep(0.02)
        
    def server_time(self):
        return time.time() - self.start_time
    
    def publish_fms(self):
        self.nt.getEntry(self.driver_station_table_key + "/isFMSAttached").setBoolean(True)
        self.nt.getEntry(self.driver_station_table_key + "/getMatchTime").setDouble(self.server_time())


DummyServer().run()