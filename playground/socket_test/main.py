#!/usr/bin/env python3

import socket
import time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        message = "%s\n" % (time.time())
        message = message.encode()
        print("Sending:", message)
        s.sendall(message)
        data = b''
        char = b''
        while char != b'\n':
            char = s.recv(1)
            data += char
        try:
            data = data.decode()
            data = float(data)
            print("Ping time: %0.4fs" % (time.time() - data))
        except ValueError:
            print('Received', repr(data))
            raise
