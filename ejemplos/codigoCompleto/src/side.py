from enum import Enum
import math

class Side(Enum):
    LEFT = 128
    RIGHT = 384
    FRONT = 256
    BACK = 0

    def get_angle(self):
        if self == Side.LEFT:
            return math.pi/2
        elif self == Side.RIGHT:
            return -math.pi/2
        return 0