from numpy import rint

from robot import Robot, MAX_VEL
from navigation.navigator import Navigator
from tile_classifier import TileClassifier, TileType
from image import ImageProcessor
from lidar import LidarProcessor, Side
from comm import Comm
from comm import MessageType
# from mapping.mapper import Mapper
from mapping.mapper_new_resolution import Mapper
# from mapping.map import TILE_SIZE
from mapping.map_new_resolution import TILE_SIZE
import math
import cv2
import utils
from visualization.visualizer import Visualizer
from vector import Vector
from collections import deque
from datetime import datetime
import statistics
import inspect

VICTIMS_SENDING_DIST = 0.052 # Wall distance when analyzing images
COGNITIVE_SENDING_DIST = 0.057
COGNITIVE_ANALYZING_DIST = 0.072
LIMIT_STEPS = 50
POS_SAMPLES = 4

BASE_TESTING_DIST = 0.17508681

ACCURACY = TILE_SIZE/4
ROBOT_RADIUS = 0.035

DEBUGGING_3_CAMS = True

class GameState:
    def __init__(self):
        self.game_score = 0
        self.time_remaining = 10000
        self.real_time_remaining = 10000
        self.battery = 100.0

class Director:
    def __init__(self, config):
        self.config = config
        self.robot = Robot()
        self.last_robot_pos = self.robot.get_position()
        self.navigator = Navigator()
        self.tile_classifier = TileClassifier()
        self.image_processor = ImageProcessor()
        self.lidar_processor = LidarProcessor()
        self.comm = Comm(self.robot)
        self.game_state = GameState()
        self.mapper=Mapper()
        self.path = None
        self.target_idx = -1
        self.img_counter = 0
        self.current_robot_area = None
        self.last_robot_area = None
        self.visualizer = Visualizer(config)

        self.step_counter = 0
        self.robot.on_step(self.on_step)

        self.detected_signs = {}
        self.pending_signs = []

        self.robot_state = None

        self.swamps_visits = {}
        
        self.grid_pos_path = []

        self.detected_obstacles = []

    def on_step(self):
        """Director's step, in which key actions are performed without making the robot class 'knowing' the director class,
        in order to mantain our code's architecture (callback algorithm)"""
        self.step_counter += 1
        self.process_messages()

        if self.step_counter % 10 == 0:
            self.comm.send_gamescore_and_time_remaining_request()

        self.navigator.mark_position_as_visited(self.robot.get_position())

        self.visualizer.step()
        self.visualizer.send_robot(self.robot)

        self.update_pending_signs()

    def __on_step(self): #DEBUG ON_STEP
        return     

    def update_pending_signs(self):
        """Updates our pending_signs list while the robot is navigating, basing on the camera's output."""
        nearby_signs = self.get_nearby_signs()
        for est_sign_pos in nearby_signs:
            if self.get_pending_sign_index(est_sign_pos) == -1:
                self.pending_signs.append(est_sign_pos)
                # print(f"Added pending sign at pos: {est_sign_pos[0]} with analyzing target pos: {est_sign_pos[1]}")

    def get_pending_sign_index(self, sign_pos):
        """With a recieved est. sign pos, we verify if the sign is already in the list or really close to an already detected one, to return its index, 
        or if its not, and the robot detected a new sign."""
        sign_index = 0
        for pending_sign_pos in self.pending_signs:
            if pending_sign_pos[0].distance_to(sign_pos[0]) < 0.0335:
                return sign_index
            sign_index += 1
        return -1
    
    def get_nearby_signs(self):
        """All sign detection/update logic."""
        return self.get_nearby_signs_with_3_cams()
    
    def get_nearby_signs_with_3_cams(self):
        """All sign detection/update logic."""
        # print("Checking nearby signs with 3 cams...")
        lidar_image = self.robot.get_lidar_image()
        robot_direction = self.robot.get_forward_vector()
        pending_signs_pos = []
        is_cognitive = False
        is_zoomed_in = False
        
        for side in Side:
            if side == Side.BACK:
                continue
            # 1) Verify if the robot is near a wall, and if it is aligned to it, to check if there is a sign in the camera's image.
            if self.lidar_processor.is_wall_at_side(lidar_image, side):
                if self.lidar_processor.is_aligned_to_wall(lidar_image, side, robot_direction):
                    point = self.get_estimated_sign_pos(lidar_image[side.value], side.get_angle())
                    if not self.is_sign_detected(point):
                        image = self.robot.get_camera_from_side(side)
                        if self.image_processor.is_victim(image):
                            is_cognitive = False
                        elif self.image_processor.is_cognitive(image):
                            is_cognitive = True
                        elif self.image_processor.is_image_zoomed_in(image):
                            if not min(lidar_image[side.value - 6:side.value + 6]) <= 0.055: 
                                continue
                            if self.image_processor.is_zoomed_image_a_victim(image): # Basing on the colours of the image, we can determine if
                                # the zoomed image is a victim or a cognitive sign.
                                is_cognitive = False
                            else:
                                is_cognitive = True
                            # print(f"Is zoomed image a victim? {not is_cognitive}")
                            is_zoomed_in = True
                        else:
                            continue
                        # self.record(special_case=False, camera=side)

                        centre = self.image_processor.get_contour_centre(image, is_cognitive, is_zoomed_in)
                        if centre is None:
                            # print(f"No contour's center found at side {side.name}!")
                            continue 
                        ray_offset = self.calculate_lidar_offset(centre - 20, lidar_image, side)
                        token_pos, target_pos = self.get_estimated_analyzing_pos(side, ray_offset, is_cognitive) # Best transitable point for the robot to analyze the sign.
                        if target_pos is None: continue
                        self.record(special_case=False, camera=side)
                        pending_signs_pos.append((token_pos, target_pos, is_cognitive))

        return pending_signs_pos

    def update_visualizer(self):
        self.visualizer.update(self)

    def __start(self): # DEBUG
        while self.robot.step():
            print("-"*30)

    def start(self):
        """Main loop of the director class, which controls the robot's actions and navigation."""
        self.current_robot_area = 1
        self.previous_robot_area = 1
        self.mapper.add_tiles({(0,0): TileType.STARTING})
        self.mapper.set_area(self.mapper.get_tile_at_pos((0,0)), self.current_robot_area)
        print(f"Initial tile at pos (0,0): {self.mapper.get_tile_at_pos((0,0))}")
        self.mapper.add_visited_tile(self.mapper.get_tile_at_pos((0,0)))

        while True:
            robot_pos = self.robot.get_position()
            abs_pos = self.robot.get_absolute_position()

            self.process_messages()
            self.update_maps()
            self.update_visits(robot_pos)
            self.update_path()
            self.update_cameras()
            self.pending_signs = []
            self.update_visualizer()
            # self.record(False)

            # If the path is empty, we break out of the loop
            if self.path is not None and len(self.path) == 0:
                self.navigator.map.increase_visit_count((0, 0), 0)
                break

            # If theres no time left, we send an early exit
            if self.game_state.time_remaining <= 10 or self.game_state.real_time_remaining <= 10:
                break

            self.move_to_target()

            # Save the robot position at the beginning of the loop in order to
            # mark this point as not traversable in case of LOP
            self.last_robot_pos = robot_pos
            self.robot.last_absolute_position = abs_pos
            if self.robot.is_lop:
                self.robot.is_lop = False

        nav_map_img = self.navigator.map.get_image()
        if nav_map_img is not None:
            cv2.imwrite("nav_map.png", nav_map_img)
            
        self.visualizer.stop()
        final_map = self.mapper.mapping_pixels(self.robot.start_pos)
        self.comm.send_map(final_map)
        self.robot.stop()
        self.robot.delay(5000)
        self.comm.send_exit()


    def move_to_target(self):
        """Moves the robot towards the current target in the path, handling obstacles and marking visited positions."""
        if self.path is None: return
        target = self.path[self.target_idx]
        if not self.robot.move_to_point(target, 0.015, blocking_steps=1): # If the robot did not arrive to dest.
            # print((f"ESTOY ATASCADO:  {self.robot.is_stuck()}"))
            if self.robot.is_stuck():
                print(f"{self.step_counter} THE ROBOT IS STUCK!")
                self.navigator.mark_position_as_obstructed(target)
                self.robot.stop()
                self.target_idx += 1
                if self.target_idx >= len(self.path):
                    self.path = None
                    self.target_idx = -1
            return

        # If execution reaches this line, we arrived to the target
        self.robot.stop()
        self.navigator.mark_position_as_visited(target)

        self.target_idx += 1
        if self.target_idx >= len(self.path):
            self.path = None
            self.target_idx = -1
        # print(f"Minitiles dict after moving to target: {self.navigator.map.minitiles}")

    def process_messages(self):
        """Processes incoming messages from the communication module, updating the game state and handling LOP events."""
        self.robot.is_lop = False
        messages = self.comm.get_messages()
        for message in messages:  #[('G', 12.2, 14, 28), ('L')]
            if message[0] == MessageType.GAME_INFO:
                self.game_state.game_score = message[1]
                self.game_state.time_remaining = message[2]
                self.game_state.real_time_remaining = message[3]
                self.game_state.battery=message[4] 
                #print(f"Battery: {self.game_state.battery}")
            elif message[0] == MessageType.LOP:
                print(f"LOP detected in process_messages! Step: {self.step_counter}")
                self.robot.is_lop = True
                self.navigator.mark_lop(self.last_robot_pos)
                self.visualizer.send_debug({"lop": self.last_robot_pos})
                self.path = None
                self.target_idx = -1
                self.robot.set_corrected_position_by_lop(self.robot.get_absolute_position(), 0.06)

    def update_maps(self):
        """Updates the maps with the robot's current position, lidar points, and surrounding tiles."""
        robot_pos = self.robot.get_position()
        lidar_points = self.robot.get_lidar_points()
        surrounding_tiles = self.tile_classifier.get_surrounding_tiles(self.robot)
        self.mapper.update(robot_pos, lidar_points, surrounding_tiles)
        self.navigator.update(robot_pos, lidar_points, surrounding_tiles)
        # obstacle_average = None
        obstacle_points = self.possible_obstacles(self.robot.lidar)
        if obstacle_points:
            obstacle_pos = obstacle_points[0]
            obstacle_relative = obstacle_pos - self.robot.start_pos
            obstacle_tile = self.tile_classifier.position_to_grid(obstacle_relative)
            self.mapper.add_obstacles(obstacle_tile)
        
        current_grid_pos = self.tile_classifier.position_to_grid(robot_pos)
        is_current_tile_visited = self.mapper.is_tile_in_visited_tiles(current_grid_pos)
        current_tile = self.mapper.get_tile_at_pos(current_grid_pos)
        if len(self.grid_pos_path) == 0 or self.grid_pos_path[-1] != current_grid_pos:
            self.grid_pos_path.append(current_grid_pos)

        # print(f"Visited tiles path: {self.grid_pos_path}")
        if not is_current_tile_visited and current_tile is not None:
            if not self.mapper.is_color_passage(current_tile):
                if len(self.mapper.visited_tiles) > 1:
                    idx = -2
                    last_grid_pos_visited = self.grid_pos_path[idx] # NOTE(Martu): The last index of the list is the current tile, we want the one before it.
                    last_added_tile = self.mapper.get_visited_tile_at_pos(last_grid_pos_visited)
                    if last_added_tile is None:
                        for visited_grid_pos in self.grid_pos_path[:-1][::-1]: # We iterate the grid pos path backwards, starting from the one before the current tile, to find the last visited tile.
                            """Here we are trying to fix the problem of the tile_classifier do not finding a surrounded tile, which causes the mapper to do not
                            add the tile to the visited tiles, and then, when we find a new tile, we do not have any reference of the area of the last tile to 
                            calculate the new tile's area. This bug raises an error, so now if we do not find any visited tile, we iterate the grid pos path backwards
                            to find the last visited tile, which should be the last tile added to the visited tiles."""
                            idx -= 1
                            last_added_tile = self.mapper.get_visited_tile_at_pos(visited_grid_pos)
                            if last_added_tile is not None:
                                break
                            else:
                                continue
                    if self.mapper.is_color_passage(last_added_tile):
                        last_area_before_passage = self.mapper.get_visited_tile_at_pos(self.grid_pos_path[idx - 1]).area
                        area = self.mapper.get_new_area(last_area_before_passage, last_added_tile)
                        current_tile.area = area
                        self.current_robot_area = area
                    else:
                        current_tile.area = last_added_tile.area
                        self.current_robot_area = last_added_tile.area
                else:
                    current_tile.area = 1
                    self.current_robot_area = 1
            else:
                current_tile.area = None
                self.current_robot_area = None

            self.mapper.add_visited_tile(current_tile)

        if current_tile is not None:
            for tile in surrounding_tiles:
                tile = self.mapper.get_tile_at_pos(tile)
                if current_tile.area is not None and tile.tile_type == TileType.BLACK_HOLE:
                    """his logic is to set area to the black hole tiles, so that they are always mapped with '*' in the matriz if they are
                    inside the area 4."""
                    tile.area = current_tile.area

    def update_path(self):
        """Updates the robot's path to the next target, recalculating it if the current target is obstructed or unreachable."""
        robot_pos = self.robot.get_position()
        target = self.path[self.target_idx] if self.path is not None else None
        if target is None or not self.navigator.is_traversable(target):
            if target is not None:
                for i in range(self.target_idx, len(self.path)):
                    self.navigator.mark_position_as_obstructed(self.path[i])
            path = self.navigator.find_path(robot_pos)
            self.path = path
            self.target_idx = 0

    def update_visits(self, robot_pos):
        """Updates the visit count for the swamps based on the robot's movement between the actual position and the last position."""
        last_minitile_vector = self.navigator.map.world_to_minitile(self.last_robot_pos)
        last_minitile = (last_minitile_vector.x, last_minitile_vector.y)
        actual_minitile_vector = self.navigator.map.world_to_minitile(robot_pos)
        actual_minitile = (actual_minitile_vector.x, actual_minitile_vector.y)
        if last_minitile != actual_minitile:
            if last_minitile in self.swamps_visits:
                # print(F"ya estuve aqui")
                increment = self.swamps_visits[last_minitile] + 1.5
                # self.swamps_visits[last_minitile] = increment
            else:
                increment = 1.5 # recomendado por Claude después de analizar el costo de los swamps, 
            # para que el robot empiece a evitarlos después de 2 visitas (siendo el costo de la visita 8 + 1.5 * visits, y el costo del desvío 24)
            self.swamps_visits[last_minitile] = increment
            self.navigator.map.increase_visit_count(last_minitile, increment)
            # print(f"LAST MINITILE {last_minitile} ------- INCREMENT {increment}")
            

    def calculate_lidar_offset(self, centre_offset, lidar_image, side):
        """Calculates the offset in rays from the centre of the camera's image to the centre of the sign's contour."""
        delta_angle = math.pi * 2 / 512
        total_angle = 27 * delta_angle # NOTE (Martu): 27 is the amount of rays inside the camera's half image (20 pixels).

        dist_offset = math.tan(total_angle) * lidar_image[side.value]
        dist_offset = (centre_offset * dist_offset) / 20
        offset_angle = math.atan2(dist_offset, lidar_image[side.value])
        rays = math.floor(offset_angle / delta_angle)
        return rays

    def update_cameras(self):
        """Updates the robot's cameras to look at the nearest detected sign, if any, and places the robot to analyze it."""
        while not len(self.pending_signs) == 0:
            pos = self.robot.get_position()
            self.pending_signs.sort(key=lambda x: x[1].distance_to(pos))
            token_point, target_pos, is_cognitive = self.pending_signs[0]

            if target_pos.distance_to(pos) > TILE_SIZE: 
                self.pending_signs = []
                return None

            # 1) Move to target pos (given by get_estimated_analyzing_pos)
            self.robot.look_at(target_pos - self.robot.get_position())
            self.robot.advance(self.robot.get_position().distance_to(target_pos), 0.002)
            # self.record()

            # 2) Look at token point (sign's centre)
            self.robot.look_at(token_point - self.robot.get_position(), 0.001)

            self.try_center_sign(analyzing_after_placing = True)
            self.try_approach_sign(is_cognitive)

            self.robot.stop()

            if self.robot.is_lop: 
                """If a lop happens in this point, we stop the analyze, so on the next step, the robot will be able to recover from it.
                Here, the robot will crash"""
                self.pending_signs = []
                return None
            
            self.record(special_case=False, camera=Side.FRONT)
            if is_cognitive:
                self.analyze_cognitive_sign()
            else:
                self.analyze_victim_sign()
            print("-"*70)
            self.pending_signs.pop(0)

    def get_estimated_sign_pos(self, lidar_dist, target_rot):
        """Estimates the position of a sign based on lidar distance and target rotation."""
        rot = self.robot.get_forward_vector().rotated(target_rot)
        sign_pos = self.robot.get_position() + rot * lidar_dist
        return sign_pos

    def is_sign_detected(self, sign_pos):
        """Checks if a sign is already detected if its position is near to an already registered sign."""
        for key in self.detected_signs.keys():
            dict_sign_pos = key[0]
            if dict_sign_pos.distance_to(sign_pos) < 0.0335:
                return True
        return False
    
    def possible_obstacles(self, lidar, distance = 0.08):
        """Checks for possible obstacles around the robot using camera data."""
        image_lidar = lidar.getRangeImage()
        layer4 = image_lidar[1536:2048]

        for side in [Side.FRONT, Side.LEFT, Side.RIGHT]:
            result = self._check_camera_obstacle(layer4, side, distance)
            # print(f"RESULTADO {result}")
            if result:
                return result
        return False
    
    def _check_camera_obstacle(self, layer4, side, distance):
        """Checks for obstacles in the camera's image, uses lidar data to estimate its position."""
        ray_index = side.value 
        ray_value = layer4[ray_index]
        # print(f"DISTANCIAAAA {ray_value}")
        if ray_value > distance:
            return False
        camera_side = self.robot.get_camera_from_side(side)
        # print(f"SIDE:  {camera_side}")
        # print(f"SIDE {side}")
        if self.image_processor.is_cognitive(camera_side) or self.image_processor.is_victim(camera_side):
            # print("IS COGNITIVE??", self.image_processor.is_cognitive(camera_side))
            # print("IS VICTIM??", self.image_processor.is_victim(camera_side))
            # print("Un sign...")
            return False
        
        obstacles_px = self.image_processor.count_continuous_obs_pixels(camera_side)
        
        if obstacles_px < 18:
            # print(f"PIXELES OBTACULOS {obstacles_px}")
            return False
        # if obstacles_px > 25:
        #     print(f"PIXELES OBTACULOS MAYORES A 25 {obstacles_px}")
            
        ray_angle = ray_index * (360.0 / 512.0)
        # angle_offset = -ray_angle # FRONT = -180, LEFT -90, RIGHT = -270
        angle_rad = math.radians(ray_angle - 180.0)

        yaw = self.robot.get_rotation()
        robot_pos = self.robot.get_position() + self.robot.start_pos

        
        x_obstacle = ray_value * math.sin(angle_rad)
        z_obstacle = ray_value * math.cos(angle_rad)

        x_final = robot_pos.x + (x_obstacle * math.cos(yaw) - z_obstacle * math.sin(yaw))
        z_final = robot_pos.y - (x_obstacle * math.sin(yaw) + z_obstacle * math.cos(yaw))
        
        final_point = Vector(x_final, z_final)
        final_point_rel = final_point - self.robot.start_pos

        for key in self.detected_signs.keys():
            dict_sign_pos = key[0]
            dict_robot_pos = key[1]
            robot_position = self.robot.get_position()
            if dict_sign_pos.distance_to(final_point_rel) < 0.065 and dict_robot_pos.distance_to(robot_position) < 0.065:
                # print(f"CLOSE TO A SIGN, PROBABLY NOT AN OBSTACLE!!.")
                return False
            
        for obstacles_position in self.detected_obstacles:
            if obstacles_position.distance_to(final_point) < 0.07:
                # print("Close to another obsacle...")
                return False
            
        # print("Final point:", final_point)
        
        self.detected_obstacles.append(final_point)
        # obstacle_points.append(final_point) 
        # print(f"FINAL POINT {[final_point]}")
        return [final_point]


    def try_center_sign(self, min_tol = 18, max_tol = 21, analyzing_after_placing = False):
        """Tries to center the sign in the image by rotating until the contour's center is within the specified tolerance range."""
        # print(f"Intentando centrar en step {self.step_counter}")
        steps = 0
        while not self.robot.is_stuck() and not self.robot.is_lop:
            # print(f"Step {steps}")
            img = self.robot.get_camera_from_side(Side.FRONT)
            centre = self.image_processor.get_contour_centre(img, analyzing_after_placing)
            if steps >= LIMIT_STEPS:
                # print(f"Could not center sign after {steps} steps, breaking.")
                break

            if centre is None: break
            centre = round(centre, 4)
            # print(f"Contour centre: {centre}")
            vel = 6.28/12
            if centre > max_tol:
                self.robot.set_velocity(vel, -vel)
            elif centre < min_tol:
                self.robot.set_velocity(-vel, vel)
            else:
                break

            if not self.robot.step():
                # print(f"Robot is stuck while trying to center sign, breaking.")
                break

            steps += 1

        self.robot.stop()

    def move_and_centre(self, distance, margin=0.001, *, blocking_steps=-1):
        """Moves the robot a specified distance while trying to center the sign in the image."""
        initial_pos = self.robot.get_position()

        is_done = False
        steps = 0
        while not self.robot.is_stuck() and not self.robot.is_lop:
            self.try_center_sign()

            traveled_dist = self.robot.get_position().distance_to(initial_pos)
            diff = abs(distance) - traveled_dist
            # print(f"Diff of move and centre: {diff}")
            if diff <= margin:
                is_done = True
                self.robot.stop()
                break

            vel = min(max(diff/0.01, 0.1), 1) * 6.28
            if distance < 0: vel *= -1

            if vel > 0 and min(self.robot.get_lidar_image()[Side.FRONT.value-45:Side.FRONT.value+45]) > 0.044:
                self.robot.set_velocity(vel, vel)
            elif vel < 0 and min(self.robot.get_lidar_image()[Side.BACK.value: Side.BACK.value+45] + self.robot.get_lidar_image()[Side.BACK.value+467:Side.BACK.value+512]) > 0.044:
                self.robot.set_velocity(vel, vel)
            else:
                self.robot.stop()
                break

            if blocking_steps == steps:
                break
            if not self.robot.step():
                break

            steps += 1

        return is_done

    def get_estimated_analyzing_pos(self, side, ray_offset, is_cognitive=False):
        """Calculates the most accurate analyzing position for the robot based on the estimated sign position."""
        central_ray = side.value
        delta_ang = -math.pi*2/512
        target_rot = side.get_angle() + ray_offset * delta_ang

        dist_a = self.robot.get_lidar_image()[central_ray + ray_offset] # Distance to the sign's center, based on the lidar's image.
        dist_b = self.robot.get_lidar_image()[central_ray + ray_offset + 2]
        if abs(dist_a) == math.inf or abs(dist_b) == math.inf:
            return None, None

        direction = self.robot.get_forward_vector().rotated(target_rot)
        a = self.robot.get_position() + (direction * dist_a)
        b = self.robot.get_position() + (direction * dist_b).rotated(delta_ang * 2) # Multiplied by the amount of rays between dist_a and dist_b.

        # print(f"b: {b}, a: {a}")
        if not is_cognitive:
            c = (b - a).normalized() * VICTIMS_SENDING_DIST
        else:
            c = (b - a).normalized() * COGNITIVE_SENDING_DIST
        c = c.rotated(-math.pi/2)
        c += a # Middle point between a and b, but rotated 90 degrees to the left, to get the best analyzing position for the robot.

        target = self.navigator.find_closest_traversable_point(c, self.robot.get_position(), True)
        if target is not None and self.get_pending_sign_index((a, target)) == -1:
            self.visualizer.send_debug({"a": a,
                            "b": b,
                            "t": target})

        if not abs(target.distance_to(a)) > 0.09:
            return a, target
        return None, None

    def try_approach_sign(self, is_cognitive=False):
        """Makes the robot approach the sign while trying to center it in the camera's image, to get a better analyzing result."""
        front_tol = None
        if is_cognitive:
            front_tol = (min(self.robot.get_lidar_image()[240: 282]) - COGNITIVE_ANALYZING_DIST)
        else:
            front_tol = (min(self.robot.get_lidar_image()[240: 282]) - VICTIMS_SENDING_DIST)
        if front_tol is not None:
            self.move_and_centre(front_tol, 0.002, blocking_steps = LIMIT_STEPS)

    def analyze_victim_sign(self):
        """Analyzes the victim sign by capturing an image and processing it."""
        token = None
        # print(f"Analyzing victim dist: {min(self.robot.get_lidar_image()[240: 272])}")
        if min(self.robot.get_lidar_image()[240: 272]) < 0.04:
            token = self.image_processor.analyze_zoomed_image(self.robot.get_camera_from_side(Side.FRONT), self.robot.get_position(), self.robot.get_forward_vector())
            # print(f"ES VICTIMA: {token}")
        else:
            # print(f"Analyzing victim sign!")
            token = self.image_processor.analyze_image(self.robot.get_camera_from_side(Side.FRONT), self.robot.get_position(), self.robot.get_forward_vector())
        if token is not None:
            print(f"Victim token: {token}")
            token_pos = self.sending_and_mapping_sign(token)
            # print("-"*60)
            return token_pos
        
    def analyze_cognitive_sign(self):
        """Analyzes the cognitive sign by capturing an image and processing it."""
        token = None
        # print(f"Analyzing cognitive sign!")
        self.try_center_sign(min_tol = 19, max_tol = 20)
        token = self.image_processor.analyze_image(self.robot.get_camera_from_side(Side.FRONT), self.robot.get_position(), self.robot.get_forward_vector())
        if min(self.robot.get_lidar_image()[240: 272]) >= 0.06:
            self.robot.advance(min(self.robot.get_lidar_image()[240: 272]) - COGNITIVE_SENDING_DIST, 0.002)
        if token is not None:
            print(f"Cognitive token: {token}")
            token_pos = self.sending_and_mapping_sign(token)
            # print("-"*60)
            return token_pos

    def sending_and_mapping_sign(self, token):
        self.send_voc_message(token)

    def send_voc_message(self, token, pos = None):
        """Sends the token and its position to the supervisor, and adds it to the mapper if it's not a fake sign."""
        self.robot.stop()
        self.robot.delay(1500)
        robot_direction = self.robot.get_forward_vector()
        absolute_pos = self.robot.get_position() + self.robot.start_pos
        relative_pos = self.robot.get_position()
        dist = self.robot.get_lidar_image()[256]
        absolute_token_pos = absolute_pos + robot_direction * dist
        relative_token_pos = relative_pos + robot_direction * dist
        if token != 'Z' and token != 'X': # If the token is not a fake sign (victim/cognitive) and a obstacle.
            self.comm.send_token(int(absolute_token_pos.x * 100), int(absolute_token_pos.y * 100), token)
            self.robot.delay(100)
            if not self.current_robot_area == 4:
                self.mapper.add_sign(token, absolute_token_pos, self.tile_classifier.position_to_grid(relative_token_pos))
        # Adding the sign (wether real or fake) to the detected_signs dictionary, so that we do not analyze it infinitely.
        self.detected_signs[(relative_token_pos, relative_pos)] = token

    def record(self, special_case = False, camera = None, type_of_image = None): # JUST FOR TESTING!
        """Records the images from the robot's cameras for testing purposes, saving them with a step counter in the erebus
        game/controllers/robot0controller subfolder."""
        if not self.config.get("record_enabled", False): return
        if camera is not None:
            if camera == Side.FRONT:
                if type_of_image is None:
                    cv2.imwrite(f"CF{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.FRONT))
                else:
                    cv2.imwrite(f"CF{str(self.step_counter).rjust(4, '0')}_{type_of_image}.png", self.robot.get_camera_from_side(Side.FRONT))
            elif camera == Side.LEFT:
                if type_of_image is None:
                    cv2.imwrite(f"CI{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.LEFT))
                else:
                    cv2.imwrite(f"CI{str(self.step_counter).rjust(4, '0')}_{type_of_image}.png", self.robot.get_camera_from_side(Side.LEFT))
            elif camera == Side.RIGHT:
                if type_of_image is None:
                    cv2.imwrite(f"CD{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.RIGHT))
                else:
                    cv2.imwrite(f"CD{str(self.step_counter).rjust(4, '0')}_{type_of_image}.png", self.robot.get_camera_from_side(Side.RIGHT))
        else:
            cv2.imwrite(f"CI{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.LEFT))
            cv2.imwrite(f"CF{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.FRONT))
            cv2.imwrite(f"CD{str(self.step_counter).rjust(4, '0')}.png", self.robot.get_camera_from_side(Side.RIGHT))