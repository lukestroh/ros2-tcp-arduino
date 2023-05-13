import socket
import time


client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client.connect(("169.254.93.101", 10002))

while True:
    msg = "<2.71>"
    client.sendall(msg.encode())
    print(msg)
    time.sleep(.01)