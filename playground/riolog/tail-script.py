from tailers import SSHTailer
import time

# "1.2.3.4" is the IP address or host name you want to access
tailer = SSHTailer("lvuser@10.0.88.2", "/home/lvuser/FRC_UserProgram.log", verbose=False)

for line in tailer.iter_whole_file():
    print(line)
try:
    while True:
        for line in tailer.tail():
            print(line)

        # wait a bit
        time.sleep(0.05)

except:
    tailer.disconnect()
