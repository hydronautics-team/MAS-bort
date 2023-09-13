import socket
import time

MESSAGE = b"Hello, World!"
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        print(f"Send `{MESSAGE}`")
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        time.sleep(1)
finally:
    sock.close()
