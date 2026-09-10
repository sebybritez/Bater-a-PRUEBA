import os
import traceback
from visualization.json_utils import JSON

COMPRESSION_ENABLED = True

if COMPRESSION_ENABLED:
    try:
        # NOTE(Richo): For compression to work we need to install the pyfastlz
        from fastlz import compress, decompress
    except:
        COMPRESSION_ENABLED = False
    
class FileConnection:
    def __init__(self, filename="replay.bin") -> None:
        self.file = None
        self.filename = filename
        try:
            os.remove(self.filename)
        except:
            pass
        self.start()

    def start(self):
        if not COMPRESSION_ENABLED: return
        self.file = open(self.filename, "ab")

    def stop(self):
        if self.file == None: return
        self.file.close()
        self.file = None

    def is_connected(self):
        return self.file != None

    def send_binary(self, type, data):
        if self.file == None: return
        try:
            type = type.to_bytes(1, byteorder="little")
            if len(data) == 0:
                count = (0).to_bytes(4, "little")
                self.file.write(type + count)
            else:
                compressed = compress(data) if COMPRESSION_ENABLED else data
                count = len(compressed).to_bytes(4, "little")
                self.file.write(type + count + compressed)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.file.close()
            self.file = None
            self.start()

    def send_json(self, type, obj):
        data = JSON.stringify(obj).encode("utf8")
        self.send_binary(type, data)