import socket
import struct
import json

class CustomSocket :

	def __init__(self,host,port) :
		self.host = host
		self.port = port
		self.sock = socket.socket()
		self.isServer = False

	def startServer(self) :
		try :
			# solve address already in use error
			# https://python-list.python.narkive.com/Y15bAxfI/socket-unbind-or-socket-unlisten-socket-error-48-address-already-in-use
			self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			self.sock.bind((self.host,self.port))
			self.sock.listen(5)
			self.isServer = True
			print("[SOCKET SERVER START AT PORT "+str(self.port)+"]")
		except Exception as e :
			print("Error :",e)
			return False
		return True

	def clientConnect(self) :
		try :
			self.sock.connect((self.host,self.port))
			print("[SOCKET CLIENT CONNECTED TO "+str(self.host)+" "+str(self.port)+"]")
		except Exception as e :
			print("Error :",e)
			return False
		return True

	def sendMsg(self,sock,msg) :
		temp = msg
		try :
			temp = msg.encode('utf-8')
		except Exception as e :
			# This message is an image
			print("[IMAGE SENT THROUGH SOCKET]")
		msg = struct.pack('>I', len(msg)) + temp
		sock.sendall(msg)

	def recvall(self,sock,n) :
		data = bytearray()
		while len(data) < n :
			packet = sock.recv(n - len(data))
			if not packet :
				return None
			data.extend(packet)
		return data

	def recvMsg(self,sock) :

		rawMsgLen = self.recvall(sock, 4)
		if not rawMsgLen :
			return None
		msgLen = struct.unpack('>I', rawMsgLen)[0]

		return self.recvall(sock, msgLen)

	def req(self, msg) :
		self.sendMsg(self.sock, msg)
		result = self.recvMsg(self.sock)
		result = result.decode('utf-8')
		return json.loads(result)

def main() :
	server = CustomSocket(socket.gethostname(), 10000)
	server.startServer()

	while True :
		conn, addr = server.sock.accept()
		print("Client connected from",addr)
		while True :
			data = server.recvMsg(conn)
			# img = np.frombuffer(data,dtype=np.uint8).reshape(720,1080,3)
			res = {"mean" : 0 , "mode" : 0 , "med" : 0}
			print(res)

if __name__ == '__main__' :
	main()

