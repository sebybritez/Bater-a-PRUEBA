import socket
import threading
import traceback
from visualization.json_utils import JSON

COMPRESSION_ENABLED = True

if COMPRESSION_ENABLED:
    try:
        # NOTE(Richo): For compression to work we need to install the pyfastlz
        from fastlz import compress, decompress
    except:
        COMPRESSION_ENABLED = False
    
class SocketConnection:
    def __init__(self, port=4321) -> None:
        self.port = port
        self.host = "127.0.0.1"
        self.socket = None
        self.start()

    def is_connected(self):
        return self.socket != None

    def start(self):
        self.thread = threading.Thread(target=self.accept_connections, args=(), daemon=True)
        self.thread.start()

    def stop(self):
        if self.socket == None: return
        self.socket.close()
        self.socket = None

    def accept_connections(self):
        print("Waiting for connections...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            self.socket = conn
            self.socket.sendall(bytes([255 if COMPRESSION_ENABLED else 0]))

    def send_binary(self, type, data):
        if self.socket == None: return
        try:
            type = type.to_bytes(1, byteorder="little")
            if len(data) == 0:
                count = (0).to_bytes(4, "little")
                self.socket.sendall(type + count)
            else:
                compressed = compress(data) if COMPRESSION_ENABLED else data
                count = len(compressed).to_bytes(4, "little")       
                self.socket.sendall(type + count + compressed)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.socket.close()
            self.socket = None
            self.start()

    def send_json(self, type, obj):
        data = JSON.stringify(obj).encode("utf8")
        self.send_binary(type, data)