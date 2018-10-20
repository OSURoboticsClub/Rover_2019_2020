import socket
import sys

# Create a UDP socket
messages = ["HELP", "LOGIN MTECH GITRDONE", "STATUS", "START", "STOP", "LOGOUT"]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = (sys.argv[1], 4547)

commands = []
commands.append(sys.argv[2])

if len(sys.argv) > 3:
    commands.append(sys.argv[3])

for command in commands:
    if command in messages:
        sent = sock.sendto(command, server_address)

        data, server = sock.recvfrom(4096)
        print data
