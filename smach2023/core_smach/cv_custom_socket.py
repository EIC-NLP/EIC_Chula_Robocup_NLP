import socket
import struct
import numpy as np
import json


class CustomSocket:

    def __init__(self, 
                 host, 
                 port,
                 log: bool = True):
        self.host = host
        self.port = port
        self.SPLITTER = b"SPLITTER"
        self.sock = socket.socket()
        self.isServer = False
        self.log = log

    def startServer(self):
        try:
            # solve address already in use error
            # https://python-list.python.narkive.com/Y15bAxfI/socket-unbind-or-socket-unlisten-socket-error-48-address-already-in-use
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.listen(5)
            self.isServer = True
            print("[SOCKET SERVER START AT PORT "+str(self.port)+"]")
        except Exception as e:
            print("Error :", e)
            return False
        return True

    def clientConnect(self):
        try:
            self.sock.connect((self.host, self.port))
            if self.log:
                print("[SOCKET CLIENT CONNECTED TO " +
                    str(self.host)+" "+str(self.port)+"]")
        except Exception as e:
            print("Error :", e)
            return False
        return True

    def sendMsg(self, sock, msg):
        temp = msg
        try:
            temp = msg.encode('utf-8')
        except Exception as e:
            # This message is an image
            if self.log:
                print("[IMAGE SENT THROUGH SOCKET]")
        msg = struct.pack('>I', len(msg)) + temp
        sock.sendall(msg)

    def recvall(self, sock, n):
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def recvMsg(self, sock, has_splitter=False):

        rawMsgLen = self.recvall(sock, 4)
        if not rawMsgLen:
            print("here")
            return None
        msgLen = struct.unpack('>I', rawMsgLen)[0]

        if has_splitter:
            return self.recvall(sock, msgLen).split(self.SPLITTER)

        return self.recvall(sock, msgLen)

    def req(self, image):
        h, w = image.shape[:-1]
        bh = bytes(str(h),'utf-8')
        bw = bytes(str(w),'utf-8')
        msg = self.SPLITTER.join((bh, bw, image.tobytes()))
        self.sendMsg(self.sock, msg)
        
        result = self.recvMsg(self.sock)
        result = result.decode('utf-8')
        return json.loads(result)

    def register(self, image, name):
        h, w = image.shape[:-1]
        bh = bytes(str(h),'utf-8')
        bw = bytes(str(w),'utf-8')
        command = b'REGISTER'
        name = str(name).encode("utf-8")
        msg = self.SPLITTER.join((bh, bw, image.tobytes(), command, name))
        self.sendMsg(self.sock, msg)
        
        result = self.recvMsg(self.sock)
        result = result.decode('utf-8')
        return json.loads(result)

    def detect(self, image):
        h, w = image.shape[:-1]
        bh = bytes(str(h),'utf-8')
        bw = bytes(str(w),'utf-8')
        command = b'DETECT'
        msg = self.SPLITTER.join((bh, bw, image.tobytes(), command))
        self.sendMsg(self.sock, msg)
        
        result = self.recvMsg(self.sock)
        result = result.decode('utf-8')
        return json.loads(result)

    def stopServer(self):
        self.sock.shutdown(socket.SHUT_RDWR)



def main():

    server = CustomSocket(socket.gethostname(), 10000)
    server.startServer()

    while True:
        conn, addr = server.sock.accept()
        print("Client connected from", addr)
        while True:
            data = server.recvMsg(conn)
            # img = np.frombuffer(data,dtype=np.uint8).reshape(720,1080,3)
            res = {"mean": 0, "mode": 0, "med": 0}
            print(res)


if __name__ == '__main__':
    main()