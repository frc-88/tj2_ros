import time
import serial
import numpy as np

serial = serial.Serial("/dev/ttyTHS0", 115200)

data = []

try:
    while True:
        serial.flush()
        send_time = time.time()
        write_data = str(send_time).encode()
        serial.write(write_data + b"\n")
        line = serial.readline()
        read_data = float(line.decode().strip())
        if send_time == read_data:
            delta = time.time() - read_data
            data.append(delta)
            print("Ping:", delta, 1.0 / delta)
        else:
            print("Return time doesn't match! %s != %s" % (send_time, read_data))
        time.sleep(0.05)
        if len(data) >= 30:
            break
finally:
    print("mean:", np.mean(data))
    print("stddev:", np.std(data))
    print("len:", len(data))
    serial.close()
