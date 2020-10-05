import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('localhost', 19090)
client_address = ('localhost',29090)
print('starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

while True:
    data, address = sock.recvfrom(4096)

    print('received {} bytes from {}'.format(
        len(data), address))
    print(data)
    msg = 'c:'+data.decode().split(':')[1]+':0:0:400:1'
    #msg = 'c:'+data.decode().split(':')[1]+':none'
    sock.sendto(msg.encode(),client_address)