import socket

UDP_IP = "172.27.183.165"
UDP_PORT = 8052

#command type 2, position (1,1,1)
MESSAGE = "2 ID 0 .1 0 .1"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
while(1):
	msg = raw_input('Enter to send\n')
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
