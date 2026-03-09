import socket

def SendToComputer():
    return SenderManager("127.0.0.1", 12345, ) 

class SenderManager:
    def __init__(self, ipaddr:str, port:int=12345):
        self.ipaddr = ipaddr
        self.port = port 
        self.s = None

    def __enter__(self):
        self.s = socket.socket()
        self.s.connect((ipaddr, port))
        return self

    def __exit__(self):
        if self.s is not None:
            self.s.close()

    def check_connected(self):
        assert self.s is not None, "NOT CONNECTED! USE\n\nwith SendToComputer() as stc:" 

    def send_bytes(self, bytes_data):
        self.check_connected()
        self.s.sendall(bytes_data)

    def send_string(self, s, encoding='ascii'):
        self.send_bytes(s.encode(encoding)) 


with SendToComputer() as stc:
    stc.send_string("HELLLO YAY") 


