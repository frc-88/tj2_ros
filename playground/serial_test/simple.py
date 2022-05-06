import sys
import time
import serial

baud = eval(sys.argv[1])

serial = serial.Serial("/dev/ttyTHS0", baud)

try:
    while True:
        # serial.flush()
        serial.write(b"BBBBBBBBBBBBBB\n")
        # time.sleep(1.0)
        print(str(serial.read(serial.inWaiting()))[2:-1])
        time.sleep(1.0)
finally:
    serial.close()
