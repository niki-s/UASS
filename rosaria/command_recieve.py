import socket

UDP_IP = "172.27.184.189"
UDP_PORT = 8054

#command type 2, position (1,1,1)
MESSAGE = "2 3 0 3"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
while(1):
	msg = raw_input('Enter to send\n')
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
