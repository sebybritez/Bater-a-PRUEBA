import numpy as np
from vector import Vector
import cv2
from vector import Vector

from tile_classifier import TileType

TILE_SIZE = 0.12
MINITILE_SIZE = TILE_SIZE/2
SCALE = 200
MINITILE_SIZE_PX = int(SCALE * MINITILE_SIZE)
LOP_INCREMENT = 2

class Map:
    def __init__(self):
        initial_size = 12
        self.img = np.ones((initial_size, initial_size), dtype=np.uint8) * 255
        self.visited_img = np.ones((initial_size, initial_size), dtype=np.uint8) * 255
        self.origin = Vector.ONE*(initial_size // 2)
        self.minitiles = {}
        self.walls = {}
        self.lop_pixels = {}
        self.active_region = {
            "top_left": Vector(self.origin.x, self.origin.y),
            "bottom_right": Vector(self.origin.x, self.origin.y)
        }

    def get_scale(self):
        return SCALE
    
    def get_image(self):
        return self.img
    
    def get_visited_image(self):
        return self.visited_img

    def world_to_pixel(self, point):
        """Transforms point from world coordinates to pixel coordinates."""
        return (point * SCALE).round()

    def pixel_to_world(self, point):
        """Transforms point from pixel coordinates to world coordinates."""
        return (point) / SCALE
    
    def minitile_to_world(self, point):
        """Transforms point from minitiles coordinates to world coordinates."""
        # print(f"Converting minitile to world: {point}")
        return point * MINITILE_SIZE
    
    def world_to_minitile(self, point):
        """Transforms point from world coordinates to minitile coordinates."""
        return (point/MINITILE_SIZE).round()
    
    def get_pixel_neighbours(self, pixel):
        """Returns the 8 neighbours of a pixel (cardinal and diagonals)."""
        directions = [
            Vector(-1, -1), Vector(-1, 0), Vector(-1, 1),  
            Vector(0, -1),                 Vector(0, 1),    
            Vector(1, -1), Vector(1, 0), Vector(1, 1)      
        ]
        result = []
        for dir in directions:
            p = pixel + dir
            result.append(p)
        return result
    
    def get_traversable_pixel_neighbours(self, pixel):
        """Iterates trough the pixel neighbours and returns only the traversable ones."""
        result = []
        for p in self.get_pixel_neighbours(pixel):
            if self.is_pixel_traversable(p):
                result.append(p)
        return result

    def is_visited(self, minitile_tuple):
        """Checks if a minitile has been visited."""
        is_on_dict, visits =  self.minitiles.get(minitile_tuple, (False, 0))
        return is_on_dict, visits
    
    def get_all_unvisited(self):
        """Returns a list of all unvisited minitiles."""
        return [pos for pos, (visited, _) in self.minitiles.items() if not visited]
    
    def is_valid(self, minitile_tuple):
        """Checks if a minitile is valid (exists in the map)."""
        return minitile_tuple in self.minitiles
        
    def is_traversable(self, point):
        """Given a point, it returns if the point's pixel is traversable."""
        pixel = self.world_to_pixel(point)
        return self.is_pixel_traversable(pixel)
    
    def is_pixel_traversable(self, pixel, special_case=False):
        """Checks if a pixel is traversable (relative to origin) based on the maps's images."""
        pixel += self.origin
        w, h = self.img.shape[0], self.img.shape[1]
        if pixel.y >= h or pixel.x >= w: return False
        if pixel.y < 0 or pixel.x < 0: return False
        return self.img[pixel.y, pixel.x] != 0 or self.visited_img[pixel.y, pixel.x] == 0
    
    def is_pixel_swamp(self, pixel):
        """Checks if a pixel is swamp (relative to origin) based on the map's image."""
        pixel += self.origin
        w, h = self.img.shape[0], self.img.shape[1]
        if pixel.y >= h or pixel.x >= w: return False
        if pixel.y < 0 or pixel.x < 0: return False
        return self.img[pixel.y, pixel.x] == 128
    
    def has_walls(self, minitile):
        """Checks if a minitile has walls."""
        return self.walls.get(minitile, False)        
        
    def find_closest_traversable_point(self, target, robot_pos, special_case=False):
        """Finds the closest traversable point of a target world pos."""
        robot_pixel = self.world_to_pixel(robot_pos)
        # print("Robot pixel:", robot_pixel)
        if not self.is_pixel_traversable(robot_pixel, special_case):
            return None
        
        target_pixel = self.world_to_pixel(target)
        while not self.is_pixel_traversable(target_pixel):
            neighbour_pixels = self.get_pixel_neighbours(target_pixel)
            target_pixel = min(neighbour_pixels, key=lambda p: p.distance_to(robot_pixel))
                
        return self.pixel_to_world(target_pixel)
            
    def mark_lop(self, position):
        """Marks a point as obstructed and expans lop if necessary."""
        self.mark_as_obstructed(position)
        lop_pixel = self.expand_lop_if_near(position)
        self.lop_pixels[lop_pixel] = 1

    def expand_lop_if_near(self, position):
        """From a position given, expands the lop if its pixel position is near an existing lop pixel."""
        lop_pixel = self.world_to_pixel(position) + self.origin
        if len(self.lop_pixels) >= 1:
            for pixel in self.lop_pixels:
                distance = pixel.distance_to(lop_pixel)
                if distance <= 15:
                    self.lop_pixels[pixel] += LOP_INCREMENT
                    self.mark_as_obstructed(pixel, self.lop_pixels[pixel], is_pixel = True) # "Key" is a pixel
        return lop_pixel
        
    def mark_as_obstructed(self, position, radius=1, is_pixel=False):
        """Sets the pixel at point as not traversable."""
        if not is_pixel:
            pixel = self.world_to_pixel(position) + self.origin
        else:
            pixel = position
        cv2.circle(self.img, (pixel.x, pixel.y), radius, 0, cv2.FILLED)
        cv2.circle(self.visited_img, (pixel.x, pixel.y), radius, 255, cv2.FILLED)

    def mark_position_as_visited(self, point):
        """Marks a point as visited in terms of pixel and minitile."""
        # print(f"Marking position as visited: {point}")
        minitile = self.world_to_minitile(point)
        if self.minitiles.get((minitile.x, minitile.y), (False, 0))[0] == False:
            self.minitiles[(minitile.x, minitile.y)] = (True, 0) # Changed structure: (minitile.x, minitile.y) -> visits
        
        robot_px = self.world_to_pixel(point) + self.origin
        self.visited_img[int(robot_px.y), int(robot_px.x)] = 0
    
    def needs_expansion(self, lidar_points, robot_position):
        """Checks if the map needs expansion based on the lidar_points or if robot is near the edge."""

        # 1. Check LiDAR points first
        for v in lidar_points:
            point = self.world_to_pixel(v + robot_position) + self.origin
            if not (0 <= point.x < self.img.shape[1] and 0 <= point.y < self.img.shape[0]):
                return True

        # 2. Now check if the robot is less than 50px from the edge
        robot_position_pixels = self.world_to_pixel(robot_position) + self.origin
        margin = 50
        if (robot_position_pixels.x < margin or
            robot_position_pixels.x >= self.img.shape[1] - margin or
            robot_position_pixels.y < margin or
            robot_position_pixels.y >= self.img.shape[0] - margin):
            return True

        return False

    def expand_map(self):
        """Expands the map if necessary."""
        new_width = self.img.shape[1] * 2
        new_height = self.img.shape[0] * 2
        new_img = np.ones((new_height, new_width), dtype=np.uint8) * 255
        new_visited = np.ones((new_height, new_width), dtype=np.uint8) * 255
        new_overlay = np.ones((new_height, new_width, 3), dtype=np.uint8) * 255
        
        new_img[new_height // 4:new_height // 4 + self.img.shape[0], new_width // 4:new_width // 4 + self.img.shape[1]] = self.img
        new_visited[new_height // 4:new_height // 4 + self.visited_img.shape[0], new_width // 4:new_width // 4 + self.visited_img.shape[1]] = self.visited_img
        
        self.img = new_img
        self.visited_img = new_visited
        self.origin = Vector.ONE*(new_width // 2)

    def update_walls(self, lidar_points, robot_position):
        """Plots lidar_points on the map texture and actualizes the map's active region."""
        radius = None
        for v in lidar_points:
            point = self.world_to_pixel(v + robot_position) + self.origin
            ray_dist = v.length() / TILE_SIZE
            if ray_dist > 4: continue
            # y= (-0.03/7) * x + 0.034
            """Calculated a linear function in which we determine the radius of the wall according to the ray dist (from one to 4 tiles), if the distance is
            less than one tile, the radius will be 0.03, and if its bigger than eight tiles, the radius is 0. Reduce the radius of the wall the further it is, 
            in order to reduce the probability of marking a pixel as not traversable when it is actually traversable, due to the inaccuracy of the gps"""
            radius = (-0.03/7) * ray_dist + 0.029
            if radius < 0:
                radius = 0
            elif radius > 0.03:
                radius = 0.03
            if 0 <= point.x < self.img.shape[1] and 0 <= point.y < self.img.shape[0]:
                cv2.circle(self.img, (point.x, point.y), int(radius*SCALE)+1, 0, cv2.FILLED)

            self.active_region = self.get_active_pixel_region(point, self.active_region)
            
            minitile = self.world_to_minitile(v + robot_position)
            self.walls[(minitile.x, minitile.y)] = True

    def get_active_pixel_region(self, pixel, active_region):
        """Updates the active pixel region based on a given lidar point."""
        pixel -= self.origin # Adjust pixel to the origin of the map (centre)

        if pixel.x < active_region["top_left"].x:
            active_region["top_left"].x = pixel.x
        if pixel.y < self.active_region["top_left"].y:
            active_region["top_left"].y = pixel.y

        if pixel.x > self.active_region["bottom_right"].x:
            active_region["bottom_right"].x = pixel.x
        if pixel.y > self.active_region["bottom_right"].y:
            active_region["bottom_right"].y = pixel.y
        return active_region
            
    def update_black_holes(self, surrounding_tiles):
        """Updates the map with black holes based on surrounding tiles types."""
        for tile, type in surrounding_tiles.items():
            if type == TileType.BLACK_HOLE:
                position = Vector(tile[0] * TILE_SIZE, tile[1] * TILE_SIZE)
                top_left = self.world_to_pixel(position - Vector.ONE*0.09) + self.origin
                bottom_right = self.world_to_pixel(position + Vector.ONE*0.09) + self.origin
                cv2.rectangle(self.img, (top_left.x, top_left.y), (bottom_right.x, bottom_right.y), 0, cv2.FILLED)

    def update_swamps(self, surrounding_tiles):
        """Updates the map with swamps based on surrounding tiles types."""
        for tile, type in surrounding_tiles.items():
            if type == TileType.SWAMP:
                position = Vector(tile[0] * TILE_SIZE, tile[1] * TILE_SIZE)
                top_left = self.world_to_pixel(position - Vector.ONE*0.06) + self.origin
                bottom_right = self.world_to_pixel(position + Vector.ONE*0.06) + self.origin
                # Swamps are represented differently in order to distinguish them from other tiles and try not to visit them
                cv2.rectangle(self.img, (top_left.x, top_left.y), (bottom_right.x, bottom_right.y), 128, cv2.FILLED)
    
    def update_visited(self, robot_position):
        """Marks the robot's minitile as visited."""
        robot_minitile = self.world_to_minitile(robot_position)
        
        robot_pixel = self.world_to_pixel(robot_position) 
        bool, visits = self.minitiles.get((robot_minitile.x, robot_minitile.y), (False, 0))
        if bool:
            self.minitiles[(robot_minitile.x, robot_minitile.y)] = (bool, visits)
        # print(f"Minitiles dict in update_visited: {self.minitiles}")

        robot_px = self.world_to_pixel(robot_position) + self.origin
        self.visited_img[int(robot_px.y), int(robot_px.x)] = 0

    def increase_visit_count(self, minitile_tuple,increment=1):
        """Increases the visit count of a minitile."""
        if minitile_tuple in self.minitiles:
            visited, visits = self.minitiles[minitile_tuple]
            self.minitiles[minitile_tuple] = (visited, visits + increment) 

    def update_minitiles(self, robot_position):
        """Marks adjacent minitiles as valid."""
        robot_minitile = self.world_to_minitile(robot_position)
        directions = []
        for dx in range(-4, 5):
            for dy in range(-4, 5):
                if dx == 0 and dy == 0: continue
                v = Vector(dx, dy)
                directions.append(v)

        for delta in directions:
            minitile = robot_minitile + delta
            minitile_pos = self.minitile_to_world(minitile)
            minitile_pixel = self.world_to_pixel(minitile_pos) + self.origin
            x1 = int(minitile_pixel.x - MINITILE_SIZE_PX/2)
            y1 = int(minitile_pixel.y - MINITILE_SIZE_PX/2)
            x2 = x1 + MINITILE_SIZE_PX
            y2 = y1 + MINITILE_SIZE_PX

            if (x1 < 0 or y1 < 0 or x2 > self.img.shape[1] or y2 > self.img.shape[0]):
                continue

            minitile_tuple = (minitile.x, minitile.y)
            is_visited, visits = self.is_visited(minitile_tuple)

            if minitile_tuple not in self.minitiles or is_visited:
                self.minitiles[minitile_tuple] = (is_visited, visits)
    
    def update(self, robot_position, lidar_points, surrounding_tiles):
        while self.needs_expansion(lidar_points, robot_position):
            self.expand_map()
        
        self.update_walls(lidar_points, robot_position)
        self.update_black_holes(surrounding_tiles)
        self.update_swamps(surrounding_tiles)
        self.update_visited(robot_position)
        self.update_minitiles(robot_position)
        
        # # self.display_map(robot_position) # DEBUG
