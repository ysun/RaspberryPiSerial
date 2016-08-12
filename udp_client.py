#!/usr/bin/env python
import socket

host = '192.168.5.157'
port = 5000
buff_size = 1024
 
addr = (host,port)

udp_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    data = raw_input('ms,A|B|R,x,y,z >>')

    if not data:
        break

    udp_client.sendto(data, addr)

    rdata,raddr = udp_client.recvfrom(buff_size)

    print str(raddr) + ":" + rdata

udp_client.close()
