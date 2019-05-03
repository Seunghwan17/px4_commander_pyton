from socket import *
from time import sleep
import struct

def coummunicator():
    while True:
        csock = socket(AF_INET, SOCK_STREAM)
        HOST = '192.168.0.12'
        PORT = 4000
        BUFSIZE = 7
        ADDR = (HOST,PORT)
        csock.connect(ADDR)
        print("connect is success")
        
        command = csock.recv(BUFSIZE, MSG_WAITALL)
        data = command.decode("UTF-8")
        print("type : {}, data len : {}, data : {}, contents : {}".format(type(command),len(command), command, data))


        to_server = int(12345)
        right_method = struct.pack("<i",to_server)
        print("Send Data : {}, bytes len : {}, bytes : {}".format(to_server, len(right_method), right_method))
        sent = csock.send(right_method)


        sleep(5)


if __name__ == '__main__':
    coummunicator()