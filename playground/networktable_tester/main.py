import time
from networktables import NetworkTables

class DummyServer:
    def __init__(self):
        NetworkTables.initialize()
        assert NetworkTables.isServer()

        self.nt = NetworkTables.getTable("swerveLibrary")

        self.config_key = "configuration/modules"

        self.start_time = time.time()
    
    def run(self):
        while True:
            for index in range(4):
                table = self.nt.getSubTable(self.config_key + "/" + str(index))
                table.getEntry("azimuthController/iMax").setDouble(72)

            time.sleep(0.5)
        
    def server_time(self):
        return time.time() - self.start_time


DummyServer().run()