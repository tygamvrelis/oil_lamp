# Networking functions
# Author: Tyler
# Date: November 4, 2019

import socket
from util import logString

UDP_PORT = 42424

def network_make_sender_sock():
    '''
    '''
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return sock

def network_send(sock, data, receiver_ip, receiver_port):
    '''
    Arguments
    --------
        host_ip : str
            IPv4 IP address of host in dotted decimal format
    '''
    sock.sendto(data, (receiver_ip, receiver_port))

def network_make_receiver_sock():
    bind_ip = '' # All addresses
    bind_port = UDP_PORT
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((bind_ip, bind_port))
    except socket.error:
        sock.bind((bind_ip, 0)) # Let OS get port dynamically
    ip, port = sock.getsockname()
    logString("Receiver listening on {0}:{1} (UDP)".format(ip, port))