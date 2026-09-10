from enum import Enum
from vector import Vector
from side import Side
import utils
import math
import cv2

DEBUGGING_3_CAMS = True

class TileType(Enum):
    BLACK_HOLE = '2'
    SWAMP = '3'
    CHECKPOINT = '4'
    STARTING = '5'
    BLUE = 'b'
    YELLOW = 'y'
    GREEN = 'g'
    PURPLE = 'p'
    ORANGE = 'o'
    RED = 'r'
    STANDARD = 's'
    UNKNOWN = 'u'
    OBSTACLE = 'x'
    CONTACT = 'c'

class TileClassifier:
    def __init__(self):
        self.counter = 0
    
    def get_surrounding_tiles(self, robot):
        """Returns a dictionary of surrounding tiles and their corresponding types based on the robot's camera images."""
        result = {}

        self.analyze_pixels(self.get_tile_pointed_by_front_camera(robot), robot.get_camera_from_side(Side.FRONT), result)
        self.analyze_pixels(self.get_tile_pointed_by_left_camera(robot), robot.get_camera_from_side(Side.LEFT), result)
        if DEBUGGING_3_CAMS:
            self.analyze_pixels(self.get_tile_pointed_by_right_camera(robot), robot.get_camera_from_side(Side.RIGHT), result)
        
        # print(f"Surrounding tiles: {result}")
        # print("-"*60)

        return result

    def position_to_grid(self, pos):
        """Converts a world position to a grid position (relative to robot's origin tile)."""
        col = round(pos.x / 0.12)
        row = round(pos.y / 0.12)
        return (col, row)
    
    def get_tile_pointed_by_front_camera(self, robot): # TODO (Martu): Optimize!!
        """Returns the tile(s) pointed by the front camera based on the robot's position, direction, and lidar image."""
        tiles = []
        robot_pos = robot.get_position()
        robot_direction = robot.get_forward_vector()
        robot_lidar_image = robot.get_lidar_image()
        # if(robot_lidar_image[256] > 0.099 and robot_lidar_image[213] > 0.099 and robot_lidar_image[299] > 0.099):
        if min(robot_lidar_image[220:292]) > 0.099:
            robot_on_horizontal_border = utils.point_on_horizontal_border(robot_pos)
            robot_on_vertical_border = utils.point_on_vertical_border(robot_pos)
            if robot_on_horizontal_border:
                # If my rotation is close to 90 or 270 degrees
                if abs(robot_direction.dot(Vector.RIGHT)) > 0.99:
                    front_camera_pointer = robot_pos + robot_direction * 0.084
                    on_vertical_border = utils.point_on_vertical_border(front_camera_pointer)
                    if not on_vertical_border:
                        front_camera_pointer = robot_pos + robot_direction.rotated(0.34) * 0.089 # Primero la tile a la izquierda
                        tile = self.position_to_grid(front_camera_pointer)
                        tiles.append(tile)
                        front_camera_pointer = robot_pos + robot_direction.rotated(-0.34) * 0.089 # Luego la tile a la derecha
                        tile = self.position_to_grid(front_camera_pointer)
                        tiles.append(tile)
                        return tiles
                return None
            elif robot_on_vertical_border:
                # If my rotation is close to 0 or 180 degrees
                if abs(robot_direction.dot(Vector.UP)) > 0.99:
                    front_camera_pointer = front_camera_pointer = robot_pos + robot_direction * 0.084
                    on_horizontal_border = utils.point_on_horizontal_border(front_camera_pointer)
                    if not on_horizontal_border:
                        front_camera_pointer = robot_pos + robot_direction.rotated(0.34) * 0.089
                        tile = self.position_to_grid(front_camera_pointer)
                        tiles.append(tile)
                        front_camera_pointer = robot_pos + robot_direction.rotated(-0.34) * 0.089
                        tile = self.position_to_grid(front_camera_pointer)
                        tiles.append(tile)
                        return tiles
                return None
            elif not robot_on_horizontal_border and not robot_on_vertical_border:
                front_camera_pointer = front_camera_pointer = robot_pos + robot_direction * 0.084
                on_horizontal_border = utils.point_on_horizontal_border(front_camera_pointer)
                on_vertical_border = utils.point_on_vertical_border(front_camera_pointer)
                if on_horizontal_border or on_vertical_border:
                    return None
                tile = self.position_to_grid(front_camera_pointer)
                tiles.append(tile)
                return tiles
        else:
            return None
        
    def get_tile_pointed_by_left_camera(self, robot):
        """Returns the tile pointed by the left camera based on the robot's position, direction, and lidar image."""
        tiles = []
        robot_pos = robot.get_position()
        robot_direction = robot.get_forward_vector()
        robot_lidar_image = robot.get_lidar_image()
        if min(robot_lidar_image[92:164]) > 0.099:
            point_cs = robot_pos + robot_direction.rotated(math.pi/2) * 0.084

            on_border_x = utils.point_on_vertical_border(point_cs)
            on_border_y = utils.point_on_horizontal_border(point_cs)

            if on_border_x or on_border_y:
                # print("No te clasifico, tile")
                return None
            
            tile = self.position_to_grid(point_cs)
            tiles.append(tile)
            return tiles
        else:
            return None
        
    def get_tile_pointed_by_right_camera(self, robot):
        """Returns the tile pointed by the right camera based on the robot's position, direction, and lidar image."""
        tiles = []
        robot_pos = robot.get_position()
        robot_direction = robot.get_forward_vector()
        robot_lidar_image = robot.get_lidar_image()
        if min(robot_lidar_image[348:420]) > 0.099:
            point_cs = robot_pos + robot_direction.rotated(-math.pi/2) * 0.084

            on_border_x = utils.point_on_vertical_border(point_cs)
            on_border_y = utils.point_on_horizontal_border(point_cs)

            if on_border_x or on_border_y:
                # print("No te clasifico, tile")
                return None
            
            tile = self.position_to_grid(point_cs)
            tiles.append(tile)
            return tiles
        else:
            return None
    
    def analyze_pixels(self, tiles, img, result):
        """Analyzes the pixels of the given tiles in the image and updates the result dictionary with their types."""
        special_case = False
        if tiles is None: return None
        # if (3,1) in tiles or (1, 5) in tiles or (1, 0) in tiles: special_case = True
        if len(tiles) == 1:
            b, g, r, _ = img[39, 20] # Front tile
            m = Floor(r, g, b).get_tile_type(tiles, special_case)
            b, g, r, _ = img[35, 20]
            w = Floor(r, g, b).get_tile_type(tiles, special_case)
            if m == w and m is not None:
                result[tiles[0]] = m
        else:
            b, g, r, _ = img[39, 0] # Left tile
            m = Floor(r, g, b).get_tile_type(tiles[0], special_case)
            b, g, r, _ = img[35, 0]
            w = Floor(r, g, b).get_tile_type(tiles[0], special_case)
            if m == w and m is not None:
                result[tiles[0]] = m

            b, g, r, _ = img[39, 39] # Right tile
            m = Floor(r, g, b).get_tile_type(tiles[1], special_case)
            b, g, r, _ = img[35, 39]
            w = Floor(r, g, b).get_tile_type(tiles[1],  special_case)
            if m == w and m is not None:
                result[tiles[1]] = m


class Floor:
    def __init__(self, r, g, b):
        # NOTE(Richo): These RGB values come from a numpy array, so we need to cast them 
        # as integer, otherwise we risk an overflow when doing arithmetic with them.
        self.red = int(r)
        self.green = int(g)
        self.blue = int(b)

    def get_tile_type(self, tile = None, special_case = False):
        """Determines the type of the tile based on its RGB values and returns the corresponding TileType."""
        if special_case: pass
        if self.is_black_hole():
            return TileType.BLACK_HOLE
        elif self.is_swamp():
            return TileType.SWAMP
        elif self.is_contact():
            return TileType.CONTACT
        elif self.is_blue():
            return TileType.BLUE
        elif self.is_green():
            return TileType.GREEN
        elif self.is_purple():
            return TileType.PURPLE
        elif self.is_red():
            return TileType.RED
        elif self.is_orange():
            return TileType.ORANGE
        elif self.is_yellow():
            return TileType.YELLOW
        elif self.is_checkpoint():
            return TileType.CHECKPOINT
        elif self.is_standard():
            return TileType.STANDARD

    def is_contact(self):
        """Detecta si el color del tile corresponde a contact=True (diffuseColor RGB: 0, 0.635, 0.635).
        En escala 0-255 recibida de la cámara: R ~ 0, G ~ 162, B ~ 162."""
        cond_255 = (self.red <= 35 and abs(self.green - 162) <= 35 and abs(self.blue - 162) <= 35 and abs(self.green - self.blue) <= 25)
        cond_norm = (self.red <= 0.15 and abs(self.green - 0.635) <= 0.15 and abs(self.blue - 0.635) <= 0.15)
        return cond_255 or cond_norm

    def is_swamp(self):
        return abs(self.red - 169) < 15 \
            and abs(self.green - 135) < 15 \
            and abs(self.blue - 75) < 15
    
    def is_black_hole(self):
        return abs(self.red) <= 30 \
            and abs(self.green) <= 30 \
            and abs(self.blue) <= 30 \
            and abs(self.red - self.green) < 2 \
            and abs(self.red - self.blue) < 2

    def is_green(self):
        return abs(self.red - 25) < 15 \
            and abs(self.green - 227) < 15 \
            and abs(self.blue - 25) < 15
    
    def is_yellow(self):
        return abs(self.red - 234) < 15 \
            and abs(self.green - 234) < 15 \
            and abs(self.blue - 47) < 15
    
    def is_red(self):
        return abs(self.red - 234) < 15 \
            and abs(self.green - 47) < 15 \
            and abs(self.blue - 47) < 15
    
    def is_blue(self):
        return abs(self.red - 45) < 5 \
            and abs(self.green - 45) < 5 \
            and abs(self.blue - 230) < 5
    
    def is_purple(self):
        return abs(self.red - 100) < 10 \
            and abs(self.green - 45) < 5 \
            and abs(self.blue - 180) < 10
    
    def is_standard(self):
        return abs(self.red - 195) < 4 \
            and abs(self.green - 195) < 4 \
            and abs(self.blue - 195) < 4
    
    def is_checkpoint(self):
        dif_green_blue = abs(self.blue-self.green)
        dif_green_red = abs(self.red-self.green)
        return dif_green_blue < 25 and dif_green_red <= 15 and self.blue > 35 \
        and self.red > 35 and self.green > 35 and self.red < 185 and self.green < 185 and self.blue < 185
        
    def is_orange(self):
        return abs(self.red - 234) < 15 \
            and abs(self.green - 188) < 15 \
            and abs(self.blue - 47) < 15
    
# floor = Floor(42, 48, 66)
# print(floor.is_checkpoint())