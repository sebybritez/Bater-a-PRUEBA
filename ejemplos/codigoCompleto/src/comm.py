import struct

class MessageType:
    GAME_INFO = 'G'  # Game information
    MAP = 'M'  # Map information
    EXIT = 'E'  # Exit message
    LOP = 'L'  # LOP message
    

class Comm:
    def __init__(self, robot):
        self.emitter = robot.emitter
        self.receiver = robot.receiver
        self.robot=robot

    def send_token(self, pos_1, pos_2, letter):
        """Sends a token message with the specified positions and letter."""
        let = bytes(letter, 'utf-8')  
        message = struct.pack("i i c", pos_1, pos_2, let) 
        self.emitter.send(message)

    def send_gamescore_and_time_remaining_request(self):
        """Sends a request for game score and time remaining."""
        message = struct.pack('c', 'G'.encode()) # message = 'G' for game information
        self.emitter.send(message) # send message     

    def send_map(self, rep):
        """Sends the map representation."""
     ## Get shape
        s = rep.shape
        ## Get shape as bytes
        s_bytes = struct.pack('2i',*s)

        ## Flattening the matrix and join with ','
        flat_map = ','.join(rep.flatten())
        ## Encode
        sub_bytes = flat_map.encode('utf-8')

        ## Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes

        ## Send map data
        self.emitter.send(a_bytes)

        #STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emitter.send(map_evaluate_request)
    
    def send_exit(self):
        """Sends an exit message to indicate the end of communication."""
        ## Exit message
        exit_mes = struct.pack('c', b'E')
        self.emitter.send(exit_mes)

    def send_LOP(self): # Not used...
        """Sends a LOP message."""
        message = struct.pack('c', 'L'.encode())
        self.emitter.send(message)

    def get_messages(self):
        """Retrieves messages from the receiver and processes them based on their type."""
        messages = []
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getBytes()
            # Check if the message is a game information message
            if message[0] == ord(MessageType.GAME_INFO):
                # Unpack the message
                _, gs, tr, rtr, bat = struct.unpack('c f i i f', message)
                messages.append((MessageType.GAME_INFO, gs, tr, rtr, bat))
            elif message[0] == ord(MessageType.LOP):
                messages.append((MessageType.LOP))
            self.receiver.nextPacket()
            
        return messages