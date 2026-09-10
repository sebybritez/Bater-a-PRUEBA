import math
from side import Side
import utils

WALL_DIST = 0.0725
ALINEATION_DIST = 0.7

class LidarProcessor:
    def is_wall_at_side(self, lidar_image, side):
        dist = lidar_image[side.value]
        # print(f"Distance at {side.name}: {dist}")
        # utils.log_debug(f"Distance at {side.name}: {dist}")
        return dist < WALL_DIST
    
    def is_aligned_to_wall(self, lidar_image, side, direction):
        delta_ang = -math.pi*2/512
        dist_a = lidar_image[side.value]
        dist_b = lidar_image[side.value + 5]
        if side == Side.LEFT:
            direction = direction.rotated(math.pi/2)
        elif side == Side.RIGHT:
            direction = direction.rotated(-math.pi/2)
        a = direction * dist_a
        b = (direction * dist_b).rotated(delta_ang * 5)
        c = (b - a).normalized()
        # utils.log_debug(f"Robot's alineation to wall at {side.name}: {c.dot(direction)}, max: {ALINEATION_DIST}")
        # print(f"Robot's alineation to wall at {side.name}: {c.dot(direction)}, max: {ALINEATION_DIST}")
        # if c.dot(direction) < ALINEATION_DIST:
        #     print(f"Robot is aligned to wall at {side.name}, alineation: {c.dot(direction)}, max: {ALINEATION_DIST}")
        return c.dot(direction) < ALINEATION_DIST