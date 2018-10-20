import socket
import sys

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('task.cstag.ca', 4547)
messages = ["HELP", "LOGIN MTECH GITRDONE", "STATUS", "START", "STOP", "LOGOUT"]

print "Connected.... Enter commands now..."
# for message in messages:
while True:
    try:
        # Send data
        message = raw_input()
        # print type(message)
        if message not in messages:
            print "Invalid command. Please try again."
            continue

        sent = sock.sendto(message, server_address)

        # Receive response
        # print 'waiting to receive'
        data, server = sock.recvfrom(4096)
        print data
    except:
        pass